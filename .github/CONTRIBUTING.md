# How to contribute to jsk_mbzirc repository

### Getting Started

- Make sure you are watching this repository.
  - By watching repository, you can track and get notification of detail information about this repository.
- Submit a ticket for your issue, assuming one does not already exist.
  - Clearly describe the issue including steps and all logs to reproduce when it seems to be a bug.
  - Make sure you fill in the earliest version that you know has the issue.
- Fork the repository on Github.

### Making Changes

- Create a topic branch from where you want to base your work.
  - This is usually the master branch.
  - Only target release branches if you are certain your fix must be on that branch.
  - To quickly create a topic branch based on master, `git checkout -b fix-foobar origin/master`. Please avoid working directly on the master branch.
- Make commits of **logical units**.
- **Check for unnecessary whitespace with `git diff --check` before committing**.
- Make sure your commit messages are in the proper format.:
  ```text
[${package}/${program_name}] short description

concrete description

Modified:
  - modified_file
...
```
- Make sure you have added not only your great changes, but also the perfect tests for them.
- Run all the tests to assure nothing else was accidentally broken.

### Coding Style

For reference, most of C++ code in jsk-ros-pkg follows the style described below.
It's not a rule, just a indication.
* Use soft tab, do not use hard tab.
* Keep 80-columns as much as possible.
* Do not write `using namespace ...` in headers.
* Do not use pointer, use `boost::shared_ptr`
* Do not use c++1x for old environment. Supporting old environment is better than
writing cool code.
* Use `@` for doxygen markdup instead of `\`

  ```c++
  /**
   * @brief
   * This is an awesome class
   **/
   class Foo
   {
     ...
   };
  ```
* if

   ```c++
   if (test) {
     awesome_code
   }
   else if (test2) {
     awesome_code
   }
   else {
     awesome_code
   }
   ```
* class

   ```c++
   class Foo: public class Bar
   {
     public:
       ...
     protected:
       bool fooBarBar();
       int foo_;
       int foo_bar_;
     private:
       ...
   };
   ```
   Class name should be camel-case and starting with upper case.
   Member variables is snake-case and should have `_` suffix.
   Method should be camel-case and starting with lower case.
* include guard

   ```c++
   #ifndef PACKAGE_NAME_HEADER_FILE_NAME_H_
   #define PACKAGE_NAME_HEADER_FILE_NAME_H_
   ...
   #endif
   ```
* Function and method

  ```c++
  int foo() {
    int bar_bar = 0;
    return 0;
  }
  ```

  Local variables in a function should be snake-case.

### Submitting Changes

- Push your changes to a topic branch in your fork of the repository.
- Submit a pull request to the repository in the `start-jsk` organization.
- After feedback has been given, your great contribution will be merged into `master` branch.

### Additional Resources

- [Github Help](https://help.github.com/)
- [rostest - ROS Wiki](http://wiki.ros.org/rostest)
- [Travis CI User Documentation](https://docs.travis-ci.com/)
