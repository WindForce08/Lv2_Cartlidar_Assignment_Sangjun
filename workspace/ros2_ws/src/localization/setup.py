from glob import glob

from setuptools import setup

setup(name='localization', version='0.1.0', packages=['localization'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/localization']),
                  ('share/localization', ['package.xml', 'README.md']),
                  ('share/localization/launch', glob('launch/*.py')),
                  ('share/localization/config', glob('config/*')),
                  ('share/localization/maps', glob('maps/*'))],
      install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
      maintainer='onchangbum', maintainer_email='dhsckdqja@gmail.com',
      description='저장 지도 위 위치 추정 · map→odom · 위치 잃음 감지와 다시 찾기', license='Apache-2.0',
      entry_points={'console_scripts': [
          'carto_start = localization.carto_start:main',
          'map_odom_relay = localization.map_odom_relay:main',
          'loc_monitor = localization.loc_monitor:main',
          'scan_body_mask = localization.scan_body_mask:main',
          'scan_merge = localization.scan_merge:main']})
