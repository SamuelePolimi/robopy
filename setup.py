from setuptools import setup

setup(name='robopy',
      version='0.1.0',
      description='Library for interfacing robots.',
      url='',
      author='Intelligent Autonomous Systems Lab and Reinforcement Learning (TU Darmstadt) and Artificial Intelligence (University of Alberta)',
      author_email='samuele.tosatto@gmail.com',
      license='MIT',
      packages=['robopy'],
      zip_safe=False,
      install_requires=[
          'numpy>=1.15.4',
          'rospkg>=1.1.10',
          # 'enum>=0.4.7',
          'scipy>=1.2.2',
          'pyusb'
      ])
