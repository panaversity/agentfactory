"""
Course Outline Database Utility

Converts course JSON files to database-ready Python dictionaries.
Handles JSON string parsing for arrays and provides database insertion helpers.

Usage:
    python course_to_db.py                    # Convert all JSON files
    python course_to_db.py AI-300_*.json      # Convert specific file
    python course_to_db.py --export-py        # Generate Python files for each course
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Any
from datetime import datetime


class CourseConverter:
    """Converts course JSON files to database-ready format."""

    def __init__(self, course_outlines_dir: Path = None):
        """Initialize converter with course outlines directory."""
        self.course_outlines_dir = course_outlines_dir or Path(__file__).parent

    def parse_json_string_field(self, json_string: str) -> Any:
        """Parse JSON string fields into Python objects."""
        try:
            return json.loads(json_string)
        except (json.JSONDecodeError, TypeError):
            return []

    def load_course_json(self, json_file: Path) -> Dict[str, Any]:
        """Load course JSON file."""
        with open(json_file, 'r', encoding='utf-8') as f:
            return json.load(f)

    def convert_to_db_format(self, course_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert course JSON to database-ready format.

        Parses JSON string fields into proper Python objects:
        - course_outcomes: string → list
        - learning_modules: string → list of dicts
        - pre_requisite: string → list
        """
        db_course = course_data.copy()

        # Parse JSON string fields
        db_course['course_outcomes'] = self.parse_json_string_field(
            course_data.get('course_outcomes', '[]')
        )
        db_course['learning_modules'] = self.parse_json_string_field(
            course_data.get('learning_modules', '[]')
        )
        db_course['pre_requisite'] = self.parse_json_string_field(
            course_data.get('pre_requisite', '[]')
        )

        # Add metadata
        db_course['converted_at'] = datetime.now().isoformat()

        return db_course

    def export_as_python(self, course_data: Dict[str, Any], output_file: Path):
        """Export course as Python file with database-ready dict."""
        db_course = self.convert_to_db_format(course_data)

        python_content = f'''"""
Course: {db_course['course_name']}
Code: {db_course['course_code']}
Generated: {db_course['converted_at']}

Database-ready course definition.
"""

# Course metadata
COURSE_CODE = "{db_course['course_code']}"
COURSE_NAME = "{db_course['course_name']}"
COURSE_INITIALS = "{db_course['course_initials']}"

# Course definition (database-ready)
course = {{
    "course_code": "{db_course['course_code']}",
    "course_name": "{db_course['course_name']}",
    "course_initials": "{db_course['course_initials']}",
    "course_description": """{db_course['course_description']}""",
    "created_by": "{db_course['created_by']}",
    "updated_by": "{db_course['updated_by']}",
    "course_outcomes": {json.dumps(db_course['course_outcomes'], indent=8)},
    "long_description": """{db_course['long_description']}""",
    "learning_modules": {json.dumps(db_course['learning_modules'], indent=8)},
    "pre_requisite": {json.dumps(db_course['pre_requisite'], indent=8)},
    "media_link": "{db_course['media_link']}"
}}


def get_course_dict():
    """Return course dictionary for database insertion."""
    return course.copy()


def get_course_outcomes():
    """Return parsed course outcomes list."""
    return course['course_outcomes']


def get_learning_modules():
    """Return parsed learning modules list."""
    return course['learning_modules']


def get_prerequisites():
    """Return parsed prerequisites list."""
    return course['pre_requisite']


# Example database insertion (adjust for your ORM/database)
def insert_to_database(db_connection):
    """
    Example database insertion function.

    Args:
        db_connection: Your database connection object

    Returns:
        Inserted course ID or None
    """
    # Example with SQLAlchemy:
    # from sqlalchemy import insert
    # stmt = insert(Course).values(**course)
    # result = db_connection.execute(stmt)
    # return result.inserted_primary_key[0]

    # Example with Django ORM:
    # from myapp.models import Course
    # course_obj = Course.objects.create(**course)
    # return course_obj.id

    # Example with raw SQL:
    # cursor = db_connection.cursor()
    # cursor.execute(
    #     \"\"\"
    #     INSERT INTO courses (course_code, course_name, ...)
    #     VALUES (%(course_code)s, %(course_name)s, ...)
    #     \"\"\",
    #     course
    # )
    # db_connection.commit()
    # return cursor.lastrowid

    pass  # Implement based on your database setup


if __name__ == "__main__":
    # Print course information
    print(f"Course Code: {{COURSE_CODE}}")
    print(f"Course Name: {{COURSE_NAME}}")
    print(f"Outcomes: {{len(get_course_outcomes())}} learning outcomes")
    print(f"Modules: {{len(get_learning_modules())}} learning modules")
    print(f"Prerequisites: {{len(get_prerequisites())}} prerequisites")
    print("\\nCourse dictionary ready for database insertion.")
    print("\\nUse get_course_dict() to get the complete course data.")
'''

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(python_content)

    def convert_all(self, export_python: bool = False) -> List[Dict[str, Any]]:
        """
        Convert all JSON course files.

        Args:
            export_python: If True, also export Python files

        Returns:
            List of database-ready course dictionaries
        """
        json_files = list(self.course_outlines_dir.glob('*.json'))
        # Exclude template file
        json_files = [f for f in json_files if not f.name.startswith('.')]

        converted_courses = []

        for json_file in json_files:
            print(f"Converting: {json_file.name}")

            # Load and convert
            course_data = self.load_course_json(json_file)
            db_course = self.convert_to_db_format(course_data)
            converted_courses.append(db_course)

            # Export as Python if requested
            if export_python:
                py_file = json_file.with_suffix('.py')
                self.export_as_python(course_data, py_file)
                print(f"  → Exported: {py_file.name}")

        return converted_courses


def main():
    """Main CLI entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Convert course JSON files to database-ready format'
    )
    parser.add_argument(
        'files',
        nargs='*',
        help='Specific JSON files to convert (default: all)'
    )
    parser.add_argument(
        '--export-py',
        action='store_true',
        help='Export as Python files'
    )
    parser.add_argument(
        '--output',
        type=str,
        help='Output file for combined courses (JSON format)'
    )

    args = parser.parse_args()

    converter = CourseConverter()

    # Convert all or specific files
    if args.files:
        converted_courses = []
        for pattern in args.files:
            files = list(converter.course_outlines_dir.glob(pattern))
            for json_file in files:
                if json_file.name.startswith('.'):
                    continue
                course_data = converter.load_course_json(json_file)
                db_course = converter.convert_to_db_format(course_data)
                converted_courses.append(db_course)

                if args.export_py:
                    py_file = json_file.with_suffix('.py')
                    converter.export_as_python(course_data, py_file)
                    print(f"Exported: {py_file}")
    else:
        converted_courses = converter.convert_all(export_python=args.export_py)

    # Output results
    print(f"\nConverted {len(converted_courses)} course(s)")

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            json.dump(converted_courses, f, indent=2, ensure_ascii=False)
        print(f"Saved to: {args.output}")

    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for course in converted_courses:
        print(f"\n{course['course_code']}: {course['course_name']}")
        print(f"  Outcomes: {len(course['course_outcomes'])}")
        print(f"  Modules: {len(course['learning_modules'])}")
        print(f"  Prerequisites: {len(course['pre_requisite'])}")


if __name__ == '__main__':
    main()
