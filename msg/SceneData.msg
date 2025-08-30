# Scene data structure for ADAS visualization
string time                 # Time in M:SS format
string ego_trans           # Ego vehicle transition (None|V33|etc)
bool lane_1                # Left lane status (true|false)
bool lane_2                # Center lane status
bool lane_3                # Right lane status

# Vehicle positions - 6 quadrants
string v11                 # Left-Front quadrant (None|car1|car2|truck1|etc)
string v12                 # Center-Front quadrant
string v13                 # Right-Front quadrant
string v21                 # Left-Rear quadrant
string v22                 # Center-Rear quadrant
string v23                 # Right-Rear quadrant

string pedest              # Pedestrian location (None|Left|Right|Center)
int32 speed                # Vehicle speed in km/h
string warning             # Warning type (None|FCW|LDW|VUW|ICW|RLV)