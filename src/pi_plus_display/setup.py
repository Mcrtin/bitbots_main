import glob

from setuptools import find_namespace_packages, find_packages, setup

package_name = "pi_plus_display"

setup(
    name=package_name,
    version="0.0.0",
    # luma/ ist eine eingebundene Kopie von luma.core (Namespace-Paket ohne __init__.py, daher eigens gesucht)
    packages=find_packages(exclude=["test"]) + find_namespace_packages(include=["luma.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="martin",
    maintainer_email="66840021+Mcrtin@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "display = pi_plus_display.display:main",
        ],
    },
)
