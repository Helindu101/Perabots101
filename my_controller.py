"""my_controller controller with camera vision and correct device access."""

from controller import Robot, Camera, Motor

if __name__ == "__main__":
    # Create the Robot instance
    robot = Robot()

    # Time step
    timestep = 64
    max_speed = 6.28

    # Initialize camera
    cam = robot.getDevice('camera')  # Use getDevice
    cam.enable(timestep)

    # Initialize motors using getDevice and casting to Motor
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')

    # Ensure they are Motor instances (Webots handles typing internally)
    assert isinstance(left_motor, Motor)
    assert isinstance(right_motor, Motor)

    # Set to velocity control
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    while robot.step(timestep) != -1:
        # Get camera image
        image = cam.getImage()
        width = cam.getWidth()
        height = cam.getHeight()
        # Get red value from two points to detect brightness/color difference
        x1 = width // 2 + 100
        x2 = width // 2 - 100
        
        r1=0
        r2=0
        for i in range(height):
            r = Camera.imageGetRed(image, width, x1, i)
            g = Camera.imageGetGreen(image, width, x1, i)
            b = Camera.imageGetBlue(image, width, x1, i)
            r1=max(r-(g+b),r1)
        for i in range(height):
            r = Camera.imageGetRed(image, width, x1, i)
            g = Camera.imageGetGreen(image, width, x1, i)
            b = Camera.imageGetBlue(image, width, x1, i)
            r2=max(r-(g+b),r2)
            

        print(f"Pixel R (Right vs Left): {r1}, {r2}")

        # Use red values to control speed: follow brightness difference
        left_speed = (r1 / 255) * max_speed +0.5*max_speed
        right_speed = (r2 / 255) * max_speed +0.5*max_speed

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)