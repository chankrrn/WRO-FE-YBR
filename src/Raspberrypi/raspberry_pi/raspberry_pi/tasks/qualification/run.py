"""
Open round entry point.

    uv run python -m tasks.qualification.run
    uv run python -m tasks.qualification.run --laps 1 --debug
"""
from tasks.cli import run_task
from tasks.qualification.task import QualificationTask

if __name__ == "__main__":
    run_task(QualificationTask)
