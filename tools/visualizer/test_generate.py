import importlib.util
import tempfile
import unittest
from pathlib import Path


SPEC = importlib.util.spec_from_file_location("visualizer_generate", Path(__file__).with_name("generate.py"))
GEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEN)


class GeneratorTest(unittest.TestCase):
    def test_parse_message_directives_and_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sample.msg").write_text(
                "# @struct Sample\n# @latest latest_sample\nuint32 sequence\nfloat32[3] value\n",
                encoding="utf-8",
            )
            messages, storage = GEN.parse_messages(root)
            self.assertEqual(messages[0]["name"], "Sample")
            self.assertEqual(messages[0]["fields"][1]["array"], "3")
            self.assertEqual(storage["latest_sample"], "Sample")

    def test_module_name(self):
        root = Path("/repo")
        self.assertEqual(GEN.module_name(root / "src/modules/chassis/file.cpp", root), "chassis")
        self.assertEqual(GEN.module_name(root / "platform/drivers/communication/can.cpp", root), "platform:can")


if __name__ == "__main__":
    unittest.main()
