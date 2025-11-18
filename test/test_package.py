#!/usr/bin/env python3
"""
Test script to verify ds_node package functionality
"""

import unittest
import subprocess
import sys
import os

class TestDsNodePackage(unittest.TestCase):
    """Test cases for ds_node package setup and basic functionality."""
    
    def setUp(self):
        """Set up test environment."""
        self.package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
    def test_package_xml_exists(self):
        """Test that package.xml exists and is valid."""
        package_xml_path = os.path.join(self.package_path, 'package.xml')
        self.assertTrue(os.path.exists(package_xml_path), "package.xml should exist")
        
        # Check that package.xml contains required fields
        with open(package_xml_path, 'r') as f:
            content = f.read()
            self.assertIn('<name>ds_node</name>', content)
            self.assertIn('<version>', content)
            self.assertIn('<description>', content)
            self.assertIn('<maintainer', content)
            self.assertIn('<license>', content)
    
    def test_cmake_lists_exists(self):
        """Test that CMakeLists.txt exists and is valid."""
        cmake_path = os.path.join(self.package_path, 'CMakeLists.txt')
        self.assertTrue(os.path.exists(cmake_path), "CMakeLists.txt should exist")
        
        with open(cmake_path, 'r') as f:
            content = f.read()
            self.assertIn('cmake_minimum_required', content)
            self.assertIn('project(ds_node', content)
            self.assertIn('find_package(ament_cmake', content)
    
    def test_source_files_exist(self):
        """Test that source files exist."""
        src_path = os.path.join(self.package_path, 'src')
        
        # Check for main source files
        expected_files = [
            'ds_node.cpp',
            'ds_convert.h',
            'ds_nmea.h'
        ]
        
        for file_name in expected_files:
            file_path = os.path.join(src_path, file_name)
            self.assertTrue(os.path.exists(file_path), f"{file_name} should exist")
    
    def test_message_files_exist(self):
        """Test that custom message files exist."""
        msg_path = os.path.join(self.package_path, 'msg')
        
        expected_messages = [
            'CompactNav.msg',
            'Kalman.msg',
            'GnssHmr.msg',
            'RawIMU.msg',
            'NmeaGGA.msg'
        ]
        
        for msg_file in expected_messages:
            msg_path_full = os.path.join(msg_path, msg_file)
            self.assertTrue(os.path.exists(msg_path_full), f"{msg_file} should exist")
    
    def test_test_files_exist(self):
        """Test that test files have been created."""
        test_path = os.path.join(self.package_path, 'test')
        
        expected_tests = [
            'test_ds_convert.cpp',
            'test_ds_nmea.cpp',
            'test_integration.cpp',
            'test_utils.hpp'
        ]
        
        for test_file in expected_tests:
            test_path_full = os.path.join(test_path, test_file)
            self.assertTrue(os.path.exists(test_path_full), f"{test_file} should exist")
    
    def test_launch_files_exist(self):
        """Test that launch files exist."""
        launch_path = os.path.join(self.package_path, 'launch')
        
        expected_launches = [
            'test_launch.py'
        ]
        
        for launch_file in expected_launches:
            launch_path_full = os.path.join(launch_path, launch_file)
            self.assertTrue(os.path.exists(launch_path_full), f"{launch_file} should exist")
    
    def test_dependencies_in_package_xml(self):
        """Test that required dependencies are in package.xml."""
        package_xml_path = os.path.join(self.package_path, 'package.xml')
        
        with open(package_xml_path, 'r') as f:
            content = f.read()
            
            # Check build dependencies
            build_deps = [
                'rclcpp',
                'std_msgs',
                'geometry_msgs',
                'sensor_msgs',
                'nav_msgs',
                'rosidl_default_generators'
            ]
            
            for dep in build_deps:
                self.assertIn(f'<depend>{dep}</depend>', content)
            
            # Check test dependencies
            test_deps = [
                'ament_cmake_gtest',
                'ament_cmake_pytest',
                'ament_lint_auto',
                'ament_lint_common'
            ]
            
            for dep in test_deps:
                self.assertIn(f'<test_depend>{dep}</test_depend>', content)
    
    def test_cmake_test_configuration(self):
        """Test that CMakeLists.txt has test configuration."""
        cmake_path = os.path.join(self.package_path, 'CMakeLists.txt')
        
        with open(cmake_path, 'r') as f:
            content = f.read()
            
            # Check for test configuration
            self.assertIn('if(BUILD_TESTING)', content)
            self.assertIn('ament_add_gtest(test_ds_convert', content)
            self.assertIn('ament_add_gtest(test_ds_nmea', content)
            self.assertIn('ament_add_gtest(test_integration', content)
    
    def test_license_file_exists(self):
        """Test that LICENSE.txt exists."""
        license_path = os.path.join(self.package_path, 'LICENSE.txt')
        self.assertTrue(os.path.exists(license_path), "LICENSE.txt should exist")
        
        with open(license_path, 'r') as f:
            content = f.read()
            self.assertIn('Apache License', content)
            self.assertIn('Version 2.0', content)
    
    def test_readme_exists(self):
        """Test that README.md exists."""
        readme_path = os.path.join(self.package_path, 'README.md')
        self.assertTrue(os.path.exists(readme_path), "README.md should exist")
        
        with open(readme_path, 'r') as f:
            content = f.read()
            self.assertIn('# DS Node', content)
            self.assertIn('## License', content)

def run_command(cmd, cwd=None):
    """Run a command and return success status."""
    try:
        result = subprocess.run(cmd, shell=True, cwd=cwd, 
                              capture_output=True, text=True, check=True)
        return True, result.stdout
    except subprocess.CalledProcessError as e:
        return False, e.stderr

if __name__ == '__main__':
    # Run basic package validation
    print("Running ds_node package validation tests...")
    
    # Test if we can find the package
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print(f"Package path: {package_path}")
    
    # Run the unit tests
    unittest.main(verbosity=2)