from setuptools import setup

package_name = 'stop_evidence_monitor'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Research prototype',
    maintainer_email='noreply@example.com',
    description='Observation-only stop outcome evidence experiments, not a safety controller',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'stop-evidence-node = stop_evidence_monitor.ros_node:main',
        'stop-evidence-demo = stop_evidence_monitor.scenarios:main',
    ]},
)
