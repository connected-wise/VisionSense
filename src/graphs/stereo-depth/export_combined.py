"""Export Fast-FoundationStereo as a single combined ONNX file.

The upstream `scripts/make_onnx.py` produces two ONNX files (feature_runner +
post_runner) because the GWC volume in between uses a Triton kernel that can't
be traced. We bypass that by calling the pure-PyTorch GWC builder, which is
already entirely standard ONNX ops (Unsqueeze / Expand / Pad / Slice / Mul /
ReduceSum / LpNormalization). The result is a single self-contained ONNX file.

Run:
  python3 cpp_demo/export_combined.py \
      --model_dir weights/23-36-37/model_best_bp2_serialize.pth \
      --save_path cpp_demo/engines \
      --height 480 --width 480 --valid_iters 8 --max_disp 192
"""

import os
import sys
import argparse

# Disable torch.compile / dynamo so the @torch.compile decorators on the volume
# builders become no-ops. On Jetson, dynamo's Triton path hits an NVML assert
# and poisons the CUDA context.
os.environ.setdefault("TORCH_COMPILE_DISABLE", "1")
os.environ.setdefault("TORCHDYNAMO_DISABLE", "1")

code_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(code_dir, ".."))

import torch
import torch.nn as nn
import torch.nn.functional as F
import yaml
from omegaconf import OmegaConf

from core.foundation_stereo import TrtFeatureRunner, TrtPostRunner, normalize_image


class GWCVolumeOp(torch.autograd.Function):
    """Custom autograd op that emits an ONNX 'GWCVolume' node.

    During tracing the forward runs the real pure-pytorch computation so the
    output shapes are correct. During ONNX export the symbolic() method emits
    a custom op node that the TRT ONNX parser maps to our GWCVolumePlugin.
    """

    @staticmethod
    def forward(ctx, left_feat, right_feat, num_groups, max_disp_quart, normalize):
        # The ONNX trace only needs correct shape — the actual math is done by
        # the TRT plugin at runtime. We still compute a real result here so the
        # dry-run sanity check is valid, but avoid F.normalize (triggers an
        # NVML assert in this Jetson PyTorch build during JIT tracing).
        B, C, H, W = left_feat.shape
        K = C // num_groups
        eps = 1e-5
        ref_g = left_feat.reshape(B, num_groups, K, H, W)
        tar_g = right_feat.reshape(B, num_groups, K, H, W)
        ref_n = ref_g / (torch.sqrt((ref_g * ref_g).sum(2, keepdim=True) + eps))
        tar_n = tar_g / (torch.sqrt((tar_g * tar_g).sum(2, keepdim=True) + eps))
        ref_n = ref_n.reshape(B, C, H, W)
        tar_n = tar_n.reshape(B, C, H, W)
        slices = []
        for d in range(max_disp_quart):
            s = F.pad(tar_n, (int(d), 0, 0, 0), "constant", 0.0)[:, :, :, :W]
            slices.append((ref_n * s).reshape(B, num_groups, K, H, W).sum(2))
        return torch.stack(slices, dim=2)

    @staticmethod
    def symbolic(g, left_feat, right_feat, num_groups, max_disp_quart, normalize):
        return g.op(
            "GWCVolume",
            left_feat, right_feat,
            num_groups_i=num_groups,
            max_disp_quart_i=max_disp_quart,
            normalize_i=normalize,
        )


def gwc_volume_plugin(left_feat, right_feat, num_groups, max_disp_quart, normalize=1):
    return GWCVolumeOp.apply(left_feat, right_feat, num_groups, max_disp_quart, normalize)


class CombinedStereo(nn.Module):
    """One module that does feature extraction + GWC volume + post-processing.

    We do NOT reuse TrtFeatureRunner because its forward calls `len(image1)`,
    which returns a python int the JIT tracer can't propagate. The downstream
    Unfold op then fails with "input size not accessible". We replicate the
    same forward inline with batch size hardcoded to 1.
    """

    def __init__(self, model):
        super().__init__()
        self.feature       = model.feature
        self.stem_2        = model.stem_2
        self.post_runner   = TrtPostRunner(model)
        self.cv_group      = model.cv_group
        self.max_disp_quart = model.args.max_disp // 4

    @torch.no_grad()
    def forward(self, left, right):
        # ---- Feature extraction (inlined TrtFeatureRunner with B=1) ----
        # Run the backbone twice rather than concat+slice. Slice on a
        # cat-produced tensor lands as aten::slice in the trace and ends up
        # carrying dynamic-shape metadata, which then breaks the downstream
        # Unfold export. Two separate forwards keeps every tensor statically
        # shaped. TRT will fuse identical sub-graphs at build time.
        left_n  = normalize_image(left)
        right_n = normalize_image(right)
        feats_left  = self.feature(left_n)
        feats_right = self.feature(right_n)
        stem_2x     = self.stem_2(left_n)

        fl04, fl08, fl16, fl32 = feats_left
        fr04 = feats_right[0]

        # ---- GWC volume — emits custom ONNX op mapped to TRT plugin ----
        gwc_volume = gwc_volume_plugin(
            fl04, fr04, self.cv_group, self.max_disp_quart, 1
        )

        # ---- Cost aggregation + GRU refinement + upsample ----
        disp = self.post_runner(fl04, fl08, fl16, fl32, fr04, stem_2x, gwc_volume)
        return disp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir",  required=True)
    parser.add_argument("--save_path",  required=True, help="Output directory.")
    parser.add_argument("--height",     type=int, default=480)
    parser.add_argument("--width",      type=int, default=480)
    parser.add_argument("--valid_iters", type=int, default=8)
    parser.add_argument("--max_disp",   type=int, default=192)
    args = parser.parse_args()

    assert args.height % 32 == 0 and args.width % 32 == 0, \
        "height and width must be divisible by 32"

    os.makedirs(args.save_path, exist_ok=True)
    torch.autograd.set_grad_enabled(False)

    print(f"[export_combined] loading {args.model_dir}")
    model = torch.load(args.model_dir, map_location="cpu", weights_only=False)
    model.args.max_disp    = args.max_disp
    model.args.valid_iters = args.valid_iters
    model.cuda().eval()

    combined = CombinedStereo(model).cuda().eval()

    left  = (torch.randn(1, 3, args.height, args.width, device="cuda") * 255).float()
    right = (torch.randn(1, 3, args.height, args.width, device="cuda") * 255).float()

    # Sanity check that the wrapper runs end-to-end on CUDA before exporting.
    print("[export_combined] dry-run forward on CUDA...")
    disp = combined(left, right)
    print(f"[export_combined] disp shape={tuple(disp.shape)} dtype={disp.dtype}")

    onnx_path = os.path.join(args.save_path, "stereo_combined.onnx")
    print(f"[export_combined] tracing -> {onnx_path}")
    torch.onnx.export(
        combined,
        (left, right),
        onnx_path,
        opset_version=17,
        input_names=["left", "right"],
        output_names=["disp"],
        do_constant_folding=True,
    )
    # The GWCVolume custom op has no ONNX shape-inference rule, so the ONNX
    # graph is missing shape annotations after the plugin node. TRT's parser
    # can't propagate shapes through the downstream If node without them.
    # Fix: inject the known output shape into the ONNX value_info.
    import onnx
    from onnx import TensorProto, helper
    m = onnx.load(onnx_path)
    h4, w4 = args.height // 4, args.width // 4
    G  = model.cv_group
    D  = args.max_disp // 4
    for node in m.graph.node:
        if node.op_type == "GWCVolume":
            vi = helper.make_tensor_value_info(
                node.output[0], TensorProto.FLOAT, [1, G, D, h4, w4]
            )
            m.graph.value_info.append(vi)
            print(f"[export_combined] injected shape [1,{G},{D},{h4},{w4}] for {node.output[0]}")
            break
    # The GWCVolume plugin makes shape inference incomplete, which causes the
    # tracer to emit an If node for `.squeeze(1)` (it doesn't know dim-1 == 1).
    # TRT rejects If nodes whose branches produce different ranks.  We know the
    # Squeeze (then) branch is always taken, so replace the If with it.
    if_nodes = [n for n in m.graph.node if n.op_type == "If"]
    for if_n in if_nodes:
        then_g = if_n.attribute[0].g  # then_branch graph
        # Inline the then-branch nodes into the main graph right before the If
        idx = list(m.graph.node).index(if_n)
        for sub_n in then_g.node:
            m.graph.node.insert(idx, sub_n)
            idx += 1
        # Rewire: map If output → then-branch output
        then_out = then_g.output[0].name
        if_out   = if_n.output[0]
        for n2 in m.graph.node:
            for i, inp in enumerate(n2.input):
                if inp == if_out:
                    n2.input[i] = then_out
        m.graph.node.remove(if_n)
        print(f"[export_combined] eliminated If node, inlined Squeeze path")

    # Run shape inference to propagate concrete shapes everywhere.
    try:
        m = onnx.shape_inference.infer_shapes(m, data_prop=True)
        print("[export_combined] shape inference OK")
    except Exception as e:
        print(f"[export_combined] shape inference partial: {e}")
    onnx.save(m, onnx_path)
    print(f"[export_combined] wrote {onnx_path} ({os.path.getsize(onnx_path)/1e6:.1f} MB)")

    cfg_path = os.path.join(args.save_path, "stereo_combined.yaml")
    cfg = OmegaConf.to_container(model.args)
    cfg["image_size"] = [args.height, args.width]
    with open(cfg_path, "w") as f:
        yaml.safe_dump(cfg, f)
    print(f"[export_combined] wrote {cfg_path}")


if __name__ == "__main__":
    main()
