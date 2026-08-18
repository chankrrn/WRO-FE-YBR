"""
Obstacle round entry point.

    uv run python -m tasks.final.run
    uv run python -m tasks.final.run --debug --laps 1
    uv run python -m tasks.final.run --no-camera     # laps only, no pillars
"""
from tasks.cli import run_task
from tasks.final.task import FinalTask

if __name__ == "__main__":
    run_task(FinalTask)
