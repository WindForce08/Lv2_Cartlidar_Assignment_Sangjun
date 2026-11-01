from setuptools import setup
setup(name='perception_survey',version='0.1.0',packages=['perception_survey'],
 data_files=[('share/ament_index/resource_index/packages',['resource/perception_survey']),('share/perception_survey',['package.xml'])],
 install_requires=['setuptools'],extras_require={'test':['pytest']},zip_safe=True,maintainer='onchangbum',maintainer_email='dhsckdqja@gmail.com',
 description='B0 퍽 관측 누적 및 조사 표',license='Apache-2.0',
 entry_points={'console_scripts':['puck_survey = perception_survey.node:main']})
