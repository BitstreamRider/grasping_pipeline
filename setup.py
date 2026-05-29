from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'grasping_pipeline'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),  glob('config/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='example@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'tester = grasping_pipeline.tester:main',
        'statemachine_tester = grasping_pipeline.statemachine_tester:main',
        'image_fetcher = grasping_pipeline.image_fetcher:main',
        'object_detector = grasping_pipeline.object_detector:main',
        'pose_estimator = grasping_pipeline.pose_estimator:main',
        'find_grasppoint_server = grasping_pipeline.grasppose_estimator:main',
        'visualizer = grasping_pipeline.visualizer:main',
        'execute_grasp_server = grasping_pipeline.execute_grasp_action_server:main',
        'place = grasping_pipeline.place:main',
        'userinput_publisher = grasping_pipeline.userinput_publisher:main',
        'statemachine = grasping_pipeline.statemachine:main'
    ],
    },

)
