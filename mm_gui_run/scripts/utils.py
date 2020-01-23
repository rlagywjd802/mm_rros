## print functions
def print_pose(pose, name="pose: "):
    pp = pose.position
    po = pose.orientation
    
    print_msg = name
    print_msg += "{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(
        pp.x, pp.y, pp.z, po.x, po.y, po.z, po.w)

    return print_msg

def print_point(point, name="point: "):    
    print_msg = name
    print_msg += "{:.2f}, {:.2f}, {:.2f}".format(point.x, point.y, point.z)

    return print_msg