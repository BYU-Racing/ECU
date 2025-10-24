# ECU
ECU for 24-25 FSAE EV Car

## Subheading
Syntax Guidelines
https://google.github.io/styleguide/cppguide.html


Core Principles

Optimize for the reader, not the writer – Code should be easy to read and maintain.
Consistency – Use consistent styles across the codebase to enable automation and reduce cognitive load.
Avoid surprising or dangerous constructs – Prefer clarity and safety over cleverness.
Concede to optimization when necessary – Performance can justify exceptions to style rules.


Function Definitions

Include one line comment before each function is defined for clarity of reader. 
Define functions in header files.


Include Order

Use angle brackets for system and standard headers.
Use quotes for project headers.
Alphabetize includes within each section.


Namespaces

Use project-specific namespaces.
Avoid using namespace directives.
Prefer named namespaces over inline or unnamed ones.


Header Files

Refer to ReadMe in include folder.


Naming Conventions

Use consistent naming for files, types, variables, constants, functions, namespaces, etc.
Avoid abbreviations unless widely understood.
Use CamelCase for types and functions, snake_case for variables, constants should be capitalized snake_case.


Comments

Include one line comment before each function is defined for clarity of reader.
Clarifiying comments within functions should be in-line.
Use comments to explain non-obvious code.
Follow consistent comment styles for files, classes, functions, and variables.
Use TODO comments with an identifier and explanation.


Formatting

Line length: 100 characters max (excluding in-line comments).
Use tabs over spaces.
Consistent formatting for function declarations, initializers, loops, conditionals, etc.
Avoid non-ASCII characters unless necessary.