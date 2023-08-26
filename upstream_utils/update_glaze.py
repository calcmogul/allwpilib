#!/usr/bin/env python3

import os
import shutil

from upstream_utils import (
    get_repo_root,
    clone_repo,
    walk_cwd_and_copy_if,
    git_am,
)


def main():
    upstream_root = clone_repo("https://github.com/stephenberry/glaze", "v1.4.0")
    wpilib_root = get_repo_root()
    wpiutil = os.path.join(wpilib_root, "wpiutil")
    os.chdir(upstream_root)

    # Delete old install
    for d in [
        "src/main/native/thirdparty/glaze/include",
    ]:
        shutil.rmtree(os.path.join(wpiutil, d), ignore_errors=True)

    # Copy glaze header files into allwpilib
    include_files = walk_cwd_and_copy_if(
        lambda dp, f: dp.startswith("./include"),
        os.path.join(wpiutil, "src/main/native/thirdparty/glaze"),
    )


if __name__ == "__main__":
    main()
