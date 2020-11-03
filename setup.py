#!/usr/bin/env python

import glob
import sys
from os import path, walk, mkdir, access, X_OK, environ, pathsep
from setuptools import setup, find_packages
import xacro
import argparse
import subprocess


def print_error(*args, **kwargs):
    """ Print in stderr. """
    print(*args, file=sys.stderr, **kwargs)


def find_resource_files():
    """ Find the files under the resource folder. """
    resource_list = []
    for (root, _, files) in walk("resource"):
        for afile in files:
            if afile != package_name:
                src = path.join(root, afile)
                dst = path.join("share", package_name, path.relpath(root, "resource"))
                resource_list.append((dst, [src]))
    return resource_list


def get_build_directory():
    if "--build-base" in sys.argv:
        index = sys.argv.index("--build-base")
    elif "-b" in sys.argv:
        index = sys.argv.index("-b")
    else:
        index = None

    if index is not None:
        if index + 1 < len(sys.argv):
            build_folder = sys.argv[index + 1]
        else:
            build_folder = "build"
    else:
        build_folder = "build"
    return build_folder


def which(program):
    """ Find program. """

    def is_exe(fpath):
        return path.isfile(fpath) and access(fpath, X_OK)

    fpath, _ = path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for env_path in environ["PATH"].split(pathsep):
            exe_file = path.join(env_path, program)
            if is_exe(exe_file):
                return exe_file

    return None


def build_xacro_files():
    """ Look for the xacro files and build then in the build folder. """

    install_list = []

    if "install" not in sys.argv:
        return install_list

    build_folder = get_build_directory()
    xacro_files = []
    for (root, _, files) in walk(path.join("resource", "xacro")):
        for afile in files:
            if afile.endswith(".urdf.xacro"):
                xacro_files.append(path.join(root, afile))

    if not path.exists(build_folder):
        mkdir(build_folder)

    for xacro_file in xacro_files:
        if which("xacro") is not None:
            for xacro_file in xacro_files:
                # Generated file name
                generated_urdf_path = path.join(
                    build_folder, "urdf", path.basename(path.splitext(xacro_file)[0])
                )
                # Call xacro.
                bash_command = ["xacro", xacro_file, "-o", generated_urdf_path]
                process = subprocess.Popen(bash_command, stdout=subprocess.PIPE)
                process.communicate()
                # Prepare the installation of the files.
                dst = path.join("share", package_name, "urdf")
                install_list.append((dst, [generated_urdf_path]))
    return install_list


# Package name.
package_name = "robot_properties_solo"

# Long description from the readme.
with open("readme.md", "r") as fh:
    long_description = fh.read()

# Install the resource files.
data_files_to_install = find_resource_files()

# Build and install the xacro files.
data_files_to_install += build_xacro_files()

# Install the amend_index files.
data_files_to_install += [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
]

# Install the package.xml.
data_files_to_install += [(path.join("share", package_name), ["package.xml"])]

# Install nodes and demos.
scripts_list = []
for (root, _, files) in walk(path.join("demos")):
    for demo_file in files:
        scripts_list.append(path.join(root, demo_file))

# Final setup.
setup(
    name=package_name,
    version="1.0.0",
    package_dir={package_name: path.join("src", package_name)},
    packages=[package_name],
    data_files=data_files_to_install,
    scripts=scripts_list,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mnaveau",
    maintainer_email="mnaveau@tuebingen.mpg.de",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    description="Wrapper around the pybullet interface using pinocchio.",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
