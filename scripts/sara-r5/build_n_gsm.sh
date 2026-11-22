#!/bin/bash
# Build and install the n_gsm (3GPP 27.010 CMUX) line discipline for the
# running kernel. The JetPack kernels ship without it (CONFIG_N_GSM is not
# set), and sara-r5-mux needs it to split the SARA-R5 UART into channels.
#
# The source comes from the kernel.org stable tree at the running kernel's
# version, built against the installed headers. Re-run after a kernel update.
set -e

KREL="$(uname -r)"
KDIR="/lib/modules/${KREL}/build"

if modinfo n_gsm >/dev/null 2>&1; then
    echo "✓ n_gsm already available for ${KREL}"
    exit 0
fi
if [ ! -f "${KDIR}/Makefile" ]; then
    echo "✗ kernel headers not found at ${KDIR}"
    exit 1
fi

# The kernel.org tag matching the headers, e.g. v6.8.12
KVER="$(awk -F' = ' '/^VERSION/{v=$2} /^PATCHLEVEL/{p=$2} /^SUBLEVEL/{s=$2} END{print "v" v "." p "." s}' "${KDIR}/Makefile")"
SRC="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/tty"

BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
echo "Building n_gsm from Linux ${KVER} for ${KREL}..."
curl -sfL -o "${BUILD}/n_gsm.c" "${SRC}/n_gsm.c?h=${KVER}"
# n_gsm.c includes the private drivers/tty/tty.h, which the headers lack
curl -sfL -o "${BUILD}/tty.h" "${SRC}/tty.h?h=${KVER}" || true
echo "obj-m := n_gsm.o" > "${BUILD}/Kbuild"
make -C "$KDIR" M="$BUILD" modules

sudo install -D -m 0644 "${BUILD}/n_gsm.ko" "/lib/modules/${KREL}/updates/n_gsm.ko"
sudo depmod -a "$KREL"
echo "✓ n_gsm installed to /lib/modules/${KREL}/updates/"
