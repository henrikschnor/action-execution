from setuptools import setup, find_packages

setup(name='action_execution',
      version='1.0.0',
      description='A robot component monitoring library',
      url='https://github.com/b-it-bots/action-execution',
      author='Alex Mitrevski',
      author_email='aleksandar.mitrevski@h-brs.de',
      keywords='action_execution robotics',
      packages=find_packages(exclude=['contrib', 'docs', 'tests']),
      project_urls={
          'Source': 'https://github.com/b-it-bots/action-execution'
      })
