"""
Manual servo test entry point.

    uv run python -m tasks.servo_test.run
    uv run python -m tasks.servo_test.run --motor-port /dev/ttyACM0
    uv run python -m tasks.servo_test.run --max-runtime 3600
"""
from tasks.cli import run_task
from tasks.servo_test.task import ServoTestTask

if __name__ == "__main__":
    run_task(ServoTestTask)
