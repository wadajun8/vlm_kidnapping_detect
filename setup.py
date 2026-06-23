from setuptools import find_packages, setup

package_name = 'vlm_kidnapping_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junya',
    maintainer_email='junya.wada.27@gmail.com',
    description='vlmで誘拐ロボット問題を検知する',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'superposition = vlm_kidnapping_detect.map_scan_superposition:main',
        ],
    },
)
