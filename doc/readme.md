\defgroup doc Documentation

# Tools used for FSW documentation

## Code documentation
Code should be documened using doxygen.

*Add here an explanation of the overall structure and how to create subsystem pages (or just provide a default template).*

## UML diagraming

Astrobee UML diagrams are created using [plantuml](http://plantuml.com/) that
transform text files into beautiful UML 2 compliant diagrams.

Specific notations used in the diagrams to represent ROS communication schemes
is described in:
[Astrobee UML Notations](doc/diagrams/notations.png)

Despite the ads overloaded website, plantuml is free and [open
source](https://github.com/plantuml/plantuml). The plantuml [Reference
Guide](http://plantuml.com/PlantUML_Language_Reference_Guide.pdf) walks to all
the diagrams types.

Plantuml relies on [Graphviz](http://www.graphviz.org/) that should be already
installed since ROS is also using this tool to render runtime node/topic graphs.
There are many ways to write/generate plantuml diagrams:
http://plantuml.com/running. Choose the integration that best suit you, but vim +
command line java works perfectly.

Note: some diagrams are not generated correctly when using graphviz 2.40 (layout
stretched vertially). So for now please use graphviz 2.38.

To generate the diagrams from the `doc\diagrams` directory:
```
make
# or to get PNG versions:
make png
```

## ROS Messages, Services and Actions

Our toolchain incudes Doxygen python filters to interpret `*.msg`, `*.action`
and `*.srv` files. Internally, the filters convert the messages, topics and
action files to markdown, and renders them in a hierarchy of Doxygen 'module'
pages. So, they will appear in the same structure as the written markdown
documentation, and not as classes / structs or other types.

In order for the files to be interpreted correctly, one needs to abide by a
couple of simple rules.
1. All header lines must begin with a '#' comment character. The end of the
   header is signified by an empty line. All comments following the newline will
   be treated as leading comments for the first declared variable.
1. Any header lines containing the word 'copyright' will be stripped from the
   documentation. All hashes ans newlines will also be stripped from the
   documentation. You are therefore discouraged from including any comment that
   relies on formatting using tabs or spaces.
1. You may document a variable above (leading) or in-line (to the right of) it's
   declaration. However, you cannot do both! An inline comment will mute a
   leading comment.

# ROS Unit Tests

The ROS test framework (rostest) is build upon the Google test framework. The
idea behind a ROS test is that it first launches a collection of ROS nodes, and
then starts the unit test, which runs tests against the ROS nodes.

Writing a unit test for a package involves modifying the package's
`CMakeLists.txt` and `package.xml` files, and writing a collection of test
cases. To see an example, have a look in the `./tools/visualeyez` folder.

To build all ROS tests, use the following:

    make -j6 tests

To execute all ROS tests run the following:

    make -j6 run_tests

Note that by default all ROS console messages (ROS_INFO, ROS_WARN, etc.) called
from within the test cases are suppressed. To print all messages, as well as a
more comprehensive test result, add the `--text` argument:

    rostest <package> <*.test> --text


Sometimes you want to debug your unit tests, but it's unclear on which port
rostest started the ROS master. A simpler approach is to manually start the ROS
master using `roscore` and then latch the rostest to this master with the
`reuse-master` argument as follows:

    rostest <package> <*.test> --reuse-master
