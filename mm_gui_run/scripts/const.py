from geometry_msgs.msg import Quaternion

FRAME_ID = "base_footprint"
EEF = "eef_pose"

GRIPPER_MESH = "package://mm_description/meshes/gripper/robotiq_2f85_opened_combined_axis_mated.STL"

INIT_R_AXIS = "z"
INIT_ORIENT = Quaternion(0.0, 0.0, -0.707107, 0.707107) # euler_to_quat(0, 0, -math.pi/2)
INIT_OFFSET = 10.0

BACK_ORIENT = Quaternion(0.0, 0.0, 0.707107, 0.707107) # euler_to_quat(0, 0, math.pi/2)

STEP1 = "Step1. Capture the point cloud"
STEP2 = "Step2. Click grasping point"
STEP3 = "Step3. Move to the clicked point"
STEP4 = "Step4. Pick the object"
STEP5 = "Step5. Move to initial pose"