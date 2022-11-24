### Drake iiwa-driver ros interface

Contains wrappers for iiwa controlling

### How to use

1. Run drake-iiwa-driver
2. Run drake_iiwa_driver_ros_interface

### TODO

    Publish:
        [ ] positions
        [ ] torques
    
    [ ] tf broadcaster        

    Subscribe:
        (in joint space)
        [ ] position
        [ ] velocity
        [ ] torque

        (in base/tcp frame)
        [ ] position
        [ ] twist
        [ ] wrench

    [ ] digital/analog ports

    [ ] *multi iiwa support