from setuptools import setup, find_packages

setup(name='gym_hsa_robot',
      version='0.0.1',
      author='James Avtges',
      packages=find_packages('gym_hsa_robot'),
      install_requires=['gym',
                        'pybullet',
                        'numpy']
)