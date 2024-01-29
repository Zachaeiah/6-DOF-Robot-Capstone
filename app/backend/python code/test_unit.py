from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *

def test_PartsDatabase():
    try:
        # Initialize PartsDatabase
        parts_db = PartsDatabase("test_parts.db")
        parts_db.create_parts_table()

        # Test add_part
        part_data = {
            'PartName': 'TestPart',
            'NumberOfParts': 5,
            'LocationX': 10.5,
            'LocationY': 7.2,
            'LocationZ': 3.0,
            'Orientation': [0.1, 0.2, 0.3],
            'FullWeight': 100.0,
            'HalfWeight': 50.0,
            'EmptyWeight': 20.0
        }
        parts_db.add_part(part_data)

        # Test get_part_by_name
        retrieved_part = parts_db.get_part_by_name('TestPart')
        assert retrieved_part is not None

        # Test print_part_info
        parts_db.print_part_info(retrieved_part)

    except AssertionError as e:
        print(f"PartsDatabase Test Assertion Error: {e}")
    except Exception as e:
        print(f"PartsDatabase Test Unexpected Error: {e}")
    finally:
        parts_db.connect()
        parts_db.cursor.execute("DROP TABLE IF EXISTS Parts")
        parts_db.conn.commit()
        parts_db.disconnect()

def test_PathPlanner():
    try:
        # Initialize PathPlanner
        path_planner = PathPlanner(1.0, 0.5)

        # Test points_on_linear_path_3d
        start_point_linear = (1, 1, 1)
        end_point_linear = (4, 4, 4)
        linear_points = path_planner.points_on_linear_path_3d(start_point_linear, end_point_linear)
        assert linear_points.shape == (21, 3)  # Assuming default precision
        assert isinstance(linear_points, np.ndarray)

        # Test points_on_circular_path_3d
        start_point_circular = (1, 0, 1)
        end_point_circular = (0, 1, 1)
        circular_points = path_planner.points_on_circular_path_3d(start_point_circular, end_point_circular)
        assert circular_points.shape == (21, 3)  # Assuming default precision
        assert isinstance(circular_points, np.ndarray)

        # Test generate_path
        path_planner.generate_path(start_point_linear, end_point_linear, True)
        path_planner.generate_path(start_point_circular, end_point_circular, False)

        # Test clear_saved_paths
        path_planner.clear_saved_paths()
        assert len(path_planner.saved_paths) == 0

    except AssertionError as e:
        print(f"PathPlanner Test Assertion Error: {e}")
    except Exception as e:
        print(f"PathPlanner Test Unexpected Error: {e}")

def test_IK():
    try:
        # Initialize RobotArm
        robot_arm = RobotArm("urdf_tes1.urdf")

        # Test calculate_ik
        target_positions = [[0.3, 0.1, 0.4], [0.1, 0.2, 0.3], [0.4, 0.3, 0.2]]
        target_orientations = [[0.2, 0.3, 0.4], [0.3, 0.2, 0.1], [0.4, 0.1, 0.2]]
        for i, ik_solution in enumerate(robot_arm.calculate_ik(target_positions, target_orientations, precision=2, batch_size=1)):
            assert len(ik_solution) == 9  # Assuming 9 DOF robot
            assert all(isinstance(angle, float) for angle in ik_solution)

            # Test if the IK solution matches the FK solution
            fk_solution = robot_arm.my_chain.forward_kinematics(np.radians(ik_solution))
            ik_result = robot_arm.calculate_ik([target_positions[i]], [target_orientations[i]], precision=2, batch_size=1)
            np.testing.assert_allclose(fk_solution[:3, 3], target_positions[i], rtol=1e-3)
            np.testing.assert_allclose(fk_solution[:3, :3], np.array(target_orientations[i]), rtol=1e-3)
            np.testing.assert_allclose(fk_solution, robot_arm.my_chain.forward_kinematics(np.radians(next(ik_result))), rtol=1e-3)

    except AssertionError as e:
        print(f"RobotArm Test Assertion Error: {e}")
    except Exception as e:
        print(f"RobotArm Test Unexpected Error: {e}")

def test_motorManager():
    try:
        # Create some motors for testing
        motors = {
            "motor1": StepperMotor("Motor 1", 1, 100.0, 10.0, 1.0, 800),
            "motor2": StepperMotor("Motor 2", 2, 200.0, 20.0, 2.0, 1600),
            "motor3": StepperMotor("Motor 3", 3, 300.0, 30.0, 3.0, 2400),
        }

        # Test the motorManager class
        manager = motorManager(motors)

        # Test activating a motor
        assert manager.activate("motor1", True) == True
        assert manager.get_motor_by_name("motor1").is_activate == True

        # Test removing a motor
        assert manager.remove_motor("motor2") == True
        assert manager.get_motor_by_name("motor2") is None

        # Test getting a motor by index
        assert manager.get_motor_by_index(3).name == "Motor 3"

        # Write and read the motor configuration
        assert manager.write_motor_config("test_config.json") == True

        # Create a new manager and read the configuration
        new_manager = motorManager({})
        assert not new_manager.motors
        assert new_manager.read_motor_config("test_config.json") == True

        # Check if all motors are still present after reading the configuration
        assert new_manager.get_motor_by_name("motor1").name == "Motor 1"
        assert new_manager.get_motor_by_name("motor3").name == "Motor 3"

    except AssertionError as e:
        print(f"motorManager Test Assertion Error: {e}")
    except Exception as e:
        print(f"motorManager Test Unexpected Error: {e}")
    finally:
        # Clean up the created test_config.json file
        import os
        if os.path.exists("test_config.json"):
            os.remove("test_config.json")

def test_Motors():
    try:
        # Test the motor class
        print("Testing Motor Class:")
        test_motor = motor("Test Motor", 1, 100.0, 10.0, 1.0)
        assert test_motor.name == "Test Motor"
        assert test_motor.index == 1
        assert test_motor.max_speed == 100.0
        assert test_motor.max_acceleration == 10.0
        assert test_motor.max_torqu == 1.0
        assert test_motor.is_activate == False

        # Test setting max speed
        test_motor.set_max_speed(150.0)
        assert test_motor.max_speed == 150.0

        # Test setting max acceleration
        test_motor.set_max_acceleration(15.0)
        assert test_motor.max_acceleration == 15.0

        # Test activating the motor
        test_motor.activate()
        assert test_motor.is_activate == True

    except AssertionError as e:
        print(f"Motor Class Test Assertion Error: {e}")
    except Exception as e:
        print(f"Motor Class Test Unexpected Error: {e}")

    try:
        print("\nTesting StepperMotor Class:")
        # Test the StepperMotor class
        test_stepper_motor = StepperMotor("Test Stepper Motor", 2, 200.0, 20.0, 2.0, 80000)
        assert test_stepper_motor.name == "Test Stepper Motor"
        assert test_stepper_motor.index == 2
        assert test_stepper_motor.max_speed == 200.0
        assert test_stepper_motor.max_acceleration == 20.0
        assert test_stepper_motor.max_torqu == 2.0
        assert test_stepper_motor.steps_per_revolution == 80000
        assert test_stepper_motor.type == "Stepper Motor"
        assert test_stepper_motor.is_activate == True

        # Test setting max speed for StepperMotor
        test_stepper_motor.set_max_speed(250.0)
        assert test_stepper_motor.max_speed == 250.0

        # Test setting max acceleration for StepperMotor
        test_stepper_motor.set_max_acceleration(25.0)
        assert test_stepper_motor.max_acceleration == 25.0

        # Test activating the StepperMotor
        test_stepper_motor.activate()
        assert test_stepper_motor.is_activate == True

    except AssertionError as e:
        print(f"StepperMotor Class Test Assertion Error: {e}")
    except Exception as e:
        print(f"StepperMotor Class Test Unexpected Error: {e}")
