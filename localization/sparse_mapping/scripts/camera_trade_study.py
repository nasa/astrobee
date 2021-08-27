import subprocess

print(
    "# fov xres yres featureerror maperror observations disterror distdev angleerror angledev"
)

resolutions = [(320, 240), (640, 480), (1280, 720)]
num_observations = [10, 20, 50]
map_error = [0.0, 0.01, 0.03, 0.05]
feature_error = [0, 1, 2, 5]
fovs = [90, 120, 150, 170]
for ferr in feature_error:
    for merr in map_error:
        for numobs in num_observations:
            for (xres, yres) in resolutions:
                for fov_x in fovs:
                    output = subprocess.check_output(
                        [
                            "./bin/evaluate_camera",
                            "--map_error",
                            str(merr),
                            "--feature_error",
                            str(ferr),
                            "--num_observations",
                            str(numobs),
                            "--fov_x",
                            str(fov_x),
                            "--xres",
                            str(xres),
                            "--yres",
                            str(yres),
                        ]
                    ).split("\n")
                    parts1 = output[0].split()
                    disterr = float(parts1[2])
                    distdev = float(parts1[4])
                    parts2 = output[1].split()
                    angerr = float(parts2[2])
                    angdev = float(parts2[4])
                    print(
                        (
                            "%g, %d, %d, %g, %g, %d, %g, %g, %g, %g"
                            % (
                                fov_x,
                                xres,
                                yres,
                                ferr,
                                merr,
                                numobs,
                                disterr,
                                distdev,
                                angerr,
                                angdev,
                            )
                        )
                    )
