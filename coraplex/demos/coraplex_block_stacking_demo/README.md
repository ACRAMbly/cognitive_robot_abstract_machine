# Block Stacking Demo

This demo shows a simple pick-and-place workflow for stacking three blocks.
It starts from the default PyCRAM setup, adds a flat table and three colored blocks,
and then uses the Tracy arms to move the blocks into a stacked arrangement.

## Files

- `demo.py`: runnable demo script
- `test_demo.py`: smoke test that imports the demo module

## Run

From this directory:

```bash
python demo.py
```

If ROS visualization is available in your environment, the demo will also publish
markers for the scene.

