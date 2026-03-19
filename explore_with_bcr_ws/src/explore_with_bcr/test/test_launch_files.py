#!/usr/bin/env python3
"""
Test cases for explore_with_bcr launch files.
Tests launch file syntax, parameters, and topic configurations.
"""

import unittest
import os
import sys
import tempfile
import yaml

# Need to source ROS2 environment before running these tests
# This allows importing launch modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class TestLaunchFileSyntax(unittest.TestCase):
    """Test that launch files have valid Python syntax."""

    def setUp(self):
        self.launch_dir = os.path.join(
            os.path.dirname(__file__), '..', 'launch'
        )

    def test_explore_with_mapping_syntax(self):
        """Test explore_with_mapping.launch.py compiles without errors."""
        launch_file = os.path.join(self.launch_dir, 'explore_with_mapping.launch.py')
        self.assertTrue(os.path.exists(launch_file), "Launch file should exist")

        # Try to compile the file
        with open(launch_file, 'r') as f:
            code = f.read()
        compile(code, launch_file, 'exec')

    def test_explore_only_syntax(self):
        """Test explore_only.launch.py compiles without errors."""
        launch_file = os.path.join(self.launch_dir, 'explore_only.launch.py')
        self.assertTrue(os.path.exists(launch_file), "Launch file should exist")

        with open(launch_file, 'r') as f:
            code = f.read()
        compile(code, launch_file, 'exec')


class TestTopicConfiguration(unittest.TestCase):
    """Test that topic configurations are correct for bcr_bot integration."""

    def setUp(self):
        self.config_file = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'tdle_params.yaml'
        )
        self.launch_dir = os.path.join(
            os.path.dirname(__file__), '..', 'launch'
        )

    def test_config_file_exists(self):
        """Test that tdle_params.yaml exists."""
        self.assertTrue(os.path.exists(self.config_file))

    def test_odom_topic_in_config(self):
        """Test that odom_topic is set to /bcr_bot/odom in config."""
        with open(self.config_file, 'r') as f:
            config = yaml.safe_load(f)

        # Check that odom_topic is configured correctly
        self.assertIn('fpoint_detector', config)
        self.assertEqual(
            config['fpoint_detector']['ros__parameters']['odom_topic'],
            '/bcr_bot/odom'
        )

        self.assertIn('subregion_arrangement', config)
        self.assertEqual(
            config['subregion_arrangement']['ros__parameters']['odom_topic'],
            '/bcr_bot/odom'
        )

        self.assertIn('fpoint_selection', config)
        self.assertEqual(
            config['fpoint_selection']['ros__parameters']['odom_topic'],
            '/bcr_bot/odom'
        )

    def test_map_topic_in_config(self):
        """Test that map_topic is set to /map in config."""
        with open(self.config_file, 'r') as f:
            config = yaml.safe_load(f)

        for node in ['fpoint_detector', 'fpoint_filter', 'subregion_arrangement', 'fpoint_selection']:
            self.assertIn(node, config)
            self.assertEqual(
                config[node]['ros__parameters']['map_topic'],
                '/map'
            )

    def test_costmap_topic_in_config(self):
        """Test that costmap_topic is set correctly."""
        with open(self.config_file, 'r') as f:
            config = yaml.safe_load(f)

        self.assertIn('fpoint_filter', config)
        self.assertEqual(
            config['fpoint_filter']['ros__parameters']['costmap_topic'],
            '/global_costmap/costmap'
        )

    def test_launch_file_has_bcr_bot_odom(self):
        """Test that launch files reference /bcr_bot/odom."""
        launch_file = os.path.join(self.launch_dir, 'explore_with_mapping.launch.py')
        with open(launch_file, 'r') as f:
            content = f.read()

        # Check for bcr_bot odometry topic
        self.assertIn('/bcr_bot/odom', content)


class TestPackageStructure(unittest.TestCase):
    """Test that the package has the expected structure."""

    def setUp(self):
        self.pkg_dir = os.path.join(os.path.dirname(__file__), '..')

    def test_required_files_exist(self):
        """Test that all required files exist."""
        required_files = [
            'CMakeLists.txt',
            'package.xml',
            'README.md',
            'launch/explore_with_mapping.launch.py',
            'launch/explore_only.launch.py',
            'config/tdle_params.yaml',
        ]

        for file_path in required_files:
            full_path = os.path.join(self.pkg_dir, file_path)
            self.assertTrue(
                os.path.exists(full_path),
                f"Required file should exist: {file_path}"
            )

    def test_cmake_installs_launch_files(self):
        """Test that CMakeLists.txt installs launch files."""
        cmake_file = os.path.join(self.pkg_dir, 'CMakeLists.txt')
        with open(cmake_file, 'r') as f:
            content = f.read()

        self.assertIn('install(DIRECTORY', content)
        self.assertIn('launch', content)
        self.assertIn('DESTINATION share', content)

    def test_package_xml_has_dependencies(self):
        """Test that package.xml declares required dependencies."""
        pkg_xml = os.path.join(self.pkg_dir, 'package.xml')
        with open(pkg_xml, 'r') as f:
            content = f.read()

        required_deps = [
            'bcr_bot',
            'tdle',
            'slam_toolbox',
            'nav2_bringup',
        ]

        for dep in required_deps:
            self.assertIn(dep, content, f"Should depend on {dep}")


class TestIntegrationPoints(unittest.TestCase):
    """Test integration between bcr_bot and TDLE."""

    def test_tdle_uses_correct_action(self):
        """Test that TDLE uses navigate_to_pose action."""
        launch_file = os.path.join(
            os.path.dirname(__file__), '..', 'launch', 'explore_with_mapping.launch.py'
        )
        with open(launch_file, 'r') as f:
            content = f.read()

        # Should include Nav2 bringup
        self.assertIn('nav2_bringup', content)

    def test_all_tdle_nodes_included(self):
        """Test that all 4 TDLE nodes are included in launch."""
        launch_file = os.path.join(
            os.path.dirname(__file__), '..', 'launch', 'explore_with_mapping.launch.py'
        )
        with open(launch_file, 'r') as f:
            content = f.read()

        required_nodes = [
            'fpoint_detector',
            'fpoint_filter',
            'subregion_arrangement',
            'fpoint_selection',
        ]

        for node in required_nodes:
            self.assertIn(f'executable=\'{node}\'', content)

    def test_slam_toolbox_included(self):
        """Test that SLAM Toolbox is included for mapping."""
        launch_file = os.path.join(
            os.path.dirname(__file__), '..', 'launch', 'explore_with_mapping.launch.py'
        )
        with open(launch_file, 'r') as f:
            content = f.read()

        self.assertIn('slam_toolbox', content)
        self.assertIn('online_async_launch.py', content)

    def test_simulation_included(self):
        """Test that Gazebo simulation is included."""
        launch_file = os.path.join(
            os.path.dirname(__file__), '..', 'launch', 'explore_with_mapping.launch.py'
        )
        with open(launch_file, 'r') as f:
            content = f.read()

        self.assertIn('gz_sim', content)
        self.assertIn('ros_gz_sim', content)


def run_tests():
    """Run all tests and return results."""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestLaunchFileSyntax))
    suite.addTests(loader.loadTestsFromTestCase(TestTopicConfiguration))
    suite.addTests(loader.loadTestsFromTestCase(TestPackageStructure))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegrationPoints))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
