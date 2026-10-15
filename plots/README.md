# Plots scripts and images

Contains the scripts used to create the plots of the report and of the main README.

- `scripts/localization_plot.py`: trajectory of a single robot, estimated and ground truth (`few_tags.bag` by default).
- `scripts/uncertainty.py`: trajectories of all the robots, target estimate and error statistics of a multi-robot run (`total_bag.bag` by default). With `--save`, or without a display, the figures are saved in this folder and a summary table is printed.

```sh
python3 plots/scripts/uncertainty.py bagfiles/total_bag.bag --save
```

`multiple.png`, `x_estimate.png`, `y_estimate.png` and `z_estimate.png` come from the current version, the other images are the ones of the report.
