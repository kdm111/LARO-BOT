from setuptools import find_packages, setup

package_name = 'arm_skills_mock'

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
    maintainer='js',
    maintainer_email='pjsu94@naver.com',
    description='팔 없이 스킬 액션 계약을 흉내내는 mock 서버(파라미터로 실패 주입)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mock_skills = arm_skills_mock.mock_skills:main'
        ],
    },
)
