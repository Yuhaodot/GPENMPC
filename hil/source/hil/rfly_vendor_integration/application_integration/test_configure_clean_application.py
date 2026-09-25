"""Offline tests for the GPENMPC firmware configure entry."""
import argparse
import importlib.util
import os
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
from unittest.mock import patch

MODULE = Path(__file__).with_name("configure_clean_application.py")
sys.path.insert(0, str(MODULE.parent))
spec = importlib.util.spec_from_file_location("configure_gpenmpc", MODULE)
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
from prepare_px4_sources import extension_prefix, extension_name_map, renamed, extension_descriptions


class ConfigureTests(unittest.TestCase):
    def setUp(self):
        output = os.environ.get("GPENMPC_TEST_OUTPUT")
        if not output:
            raise RuntimeError("Set GPENMPC_TEST_OUTPUT to a private test output directory")
        base = Path(output).resolve()
        base.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="configure-test-", dir=base)
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_output_rejects_source_and_ancestors(self):
        source = self.root/"source"
        for output in [source, source/"build", self.root]:
            with self.assertRaises(ValueError):
                config.validate_output(output, [source])
        config.validate_output(self.root/"output", [source])

    def test_output_requires_absolute_path(self):
        with self.assertRaises(argparse.ArgumentTypeError):
            config.output_path("relative")

    def test_cmake_path_preserves_spaces(self):
        self.assertEqual(config.cmake_path(Path("/tmp/project name")), "/tmp/project name")
        for value in ['x"y', "x;y", "x\ny", "x$y"]:
            with self.assertRaises(ValueError):
                config.cmake_path(Path(value))

    def test_copy_completes_partial_source(self):
        source, target = self.root/"source", self.root/"private"
        source.mkdir()
        (source/"Makefile").write_text("input\n")
        (source/"required.c").write_text("int value;\n")
        target.mkdir()
        (target/"Makefile").write_text("prepared\n")
        listing = b"100644 abc 0\tMakefile\0"+"100644 abc 0\trequired.c\0".encode()
        with patch.object(config.subprocess, "check_output", return_value=listing):
            count = config.copy_tracked(source, target, source, target, {})
        self.assertEqual(count, 1)
        self.assertEqual((target/"Makefile").read_text(), "prepared\n")
        self.assertEqual((target/"required.c").read_text(), "int value;\n")

    def test_missing_source_requires_readable_git_object(self):
        source = self.root/"source"
        source.mkdir()
        with patch.object(config.subprocess, "check_output", side_effect=[
                b"100644 abc 0\tmissing.c\0", subprocess.CalledProcessError(1, "git")]):
            with self.assertRaises(subprocess.CalledProcessError):
                config.copy_tracked(source, self.root/"private", source,
                                    self.root/"private", {})

    def test_missing_tracked_file_is_restored_from_git(self):
        source = self.root/"source"
        source.mkdir()
        restored = []
        with patch.object(config.subprocess, "check_output", side_effect=[
                b"100644 abc 0\tLICENSE\0", b"original license\n"]):
            count = config.copy_tracked(source, self.root/"private", source,
                                        self.root/"private", {}, restored)
        self.assertEqual(count, 1)
        self.assertEqual((self.root/"private/LICENSE").read_bytes(), b"original license\n")
        self.assertEqual(restored[0]["git_blob"], "abc")

    def test_submodule_index_entry_is_not_a_file(self):
        source = self.root/"source"
        source.mkdir()
        with patch.object(config.subprocess, "check_output",
                          return_value=b"160000 abc 0\tnested\0"):
            self.assertEqual(config.copy_tracked(source, self.root/"private",
                                                source, self.root/"private", {}), 0)

    def test_package_copy_excludes_build_products(self):
        source = self.root/"package"
        (source/"build").mkdir(parents=True)
        for name in ["CMakeLists.txt", "source.c", "cached.o", "library.a", "build/cache.txt"]:
            (source/name).write_text(name)
        destination = self.root/"private"
        self.assertEqual(config.copy_package_tree(source, destination), 2)
        self.assertTrue((destination/"CMakeLists.txt").is_file())
        self.assertFalse((destination/"cached.o").exists())
        self.assertFalse((destination/"build/cache.txt").exists())

    def test_command_uses_explicit_roots(self):
        args = argparse.Namespace(px4_root=self.root/"px4", matlab_root=self.root/"matlab",
                                  output_root=self.root/"output", board_uid=0x1122334455667788)
        command = config.configure_command(args, self.root/"integration")
        self.assertIn(str(args.px4_root), command)
        self.assertIn(str(args.output_root/"build"), command)
        self.assertIn("-DGPENMPC_MATLAB_INCLUDE="+config.cmake_path(args.matlab_root/"extern/include"), command)
        self.assertIn("-DGPENMPC_BOARD_UID=1234605616436508552", command)
        self.assertFalse(any("OLD_BUILD" in value or "compile_commands.json" in value
                             for value in command))

    def test_board_uid_requires_nonzero_uint64(self):
        self.assertEqual(config.board_uid("0x1122334455667788"), 0x1122334455667788)
        self.assertEqual(config.board_uid("18446744073709551615"), 2**64-1)
        for value in ["0", "-1", "18446744073709551616", "auto", "", "1;2"]:
            with self.assertRaises(argparse.ArgumentTypeError):
                config.board_uid(value)

    def test_extension_pair_detection(self):
        for module in ['delivery_se3_control', 'delivery_trajectory_exec']:
            path = self.root/'src/modules'/module
            path.mkdir(parents=True)
            (path/'CMakeLists.txt').write_text('')
        self.assertEqual(extension_prefix(self.root), 'delivery')

    def test_extension_pair_requires_unique_match(self):
        with self.assertRaises(ValueError):
            extension_prefix(self.root)
        for prefix in ['delivery', 'tracking']:
            for suffix in ['se3_control', 'trajectory_exec']:
                path = self.root/'src/modules'/(prefix+'_'+suffix)
                path.mkdir(parents=True)
                (path/'CMakeLists.txt').write_text('')
        with self.assertRaises(ValueError):
            extension_prefix(self.root)

    def test_extension_names_are_consistent(self):
        mapping = extension_name_map('flight_control')
        self.assertEqual(renamed('FLIGHT_CONTROL Flight_Control flight_control '
                                 'flight-control FLIGHTCONTROL FlightControl flightcontrol', mapping),
                         'GPENMPC GPENMPC gpenmpc gpenmpc GPENMPC GPENMPC gpenmpc')
        self.assertEqual(extension_name_map('gpenmpc'), [])

    def test_extension_display_labels_preserve_parameter_definition(self):
        mapping = extension_name_map('flight_control')
        text = '/** Flight Control N0 nominal controller; Flight Control HIL experiment. */\nPARAM_DEFINE_INT32(RA_CTRL_MODE, 0);'
        mapped = renamed(text, mapping)
        result = extension_descriptions(mapped, 'src/modules/gpenmpc_se3_control/control_params.c')
        self.assertEqual(result, '/** GPENMPC nominal controller; GPENMPC HIL application. */\nPARAM_DEFINE_INT32(RA_CTRL_MODE, 0);')
        self.assertEqual(extension_descriptions(mapped, 'src/modules/other/source.c'), mapped)


if __name__ == "__main__":
    unittest.main(verbosity=2)
