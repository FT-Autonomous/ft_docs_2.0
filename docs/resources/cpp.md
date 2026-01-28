# C++ resources

C++ is generally the go-to language for real-time performance, but you have to have a very good knowledge of how to go about writing programs. Here are a few general tips and resources to hopefully give yourself a better time, and to hopefully remember for the next time you feel your in a rut.

## General advice:


### Use an IDE

I can clearly remember the amount of time I spent using VS code to program in C++ and whilst you can get quite far, if you have an IDE (such as CLion) configured correctly, your life will be quite a bit easier.

And I'm not just talking about better syntax highlighting, assuming you have the right header files in the file your working on, you can **select and manage complex types quite effectively**, not to mention finding out about a few more you didn't know about.

You want to be able to get the most out of the framework your working with as you possibly can, so getting a mini dropdown menu of all the message types you have at your disposal is very useful.

Personally I recommend **CLion (from IntelliJ)**. It might have been the only one I've used, but so far its worked fine on linux, and hasn't given me too much grief.

### Semicolons everywhere

If there's a piece of code you just can't understand is wrong, and you've quoted GPT a million times to no avail, **it might be because you missed a semicolon on the line prior**. Its happened too many times to me, and C++ compilers generally not built to spot that type of thing (you might think they should be, but think about it and it might make sense why they aren't), and neither is an IDE (to my knowledge).

### Start small

Try keep your program basic, and **get it to compile first**. I know from experience with ROS2, that it doesn't handle poorly defined messaged types too well, to the point where I've just gotten a terminal full of complete garbage because of a type mismatch between ConstPointArray instead of PointArray (something about ROS2's poor handling of volatile types, according to GPT). That makes finding the actual basic errors you need to eliminate first even more frustrating that they should be.

There are plenty of github pages online detailiing simple ROS2 nodes in both Python and C++, so I'd recommend going based on one of them first if your starting from scratch (try to match the ROS2 version as well!).

## Advanced C++ methods:

### Constexpr

```constexpr``` is a great tool to increase the performance of your code. Everytime you want to run a C++ program, you need to compile it first. But in some cases, where a lot of the numbers and math is defined, **some functions or expression outputs can be worked out ahead of time**.

I.e. If you have some complex math functions that can be evaluated ahead of time, but rely on some predifined constants, **you can bascially get the compiler to work it all out for you ahead of runtime** with constexpr.

In the context of an autonomous car, there are a lot of realtime variables, meaning constexpr isn't that useful, but if there's some pieces of math that relies purely on constants in your program ahead of runtime, constexpr is definetly worth looking at, and can save a ton of performance.

E.g. If I define my lookahead circle radius to be ```5 * LOOK_AHEAD_CONSTANT```, then I can do ```constexpr int LOOKAHEAD_RADIUS = 5 * LOOK_AHEAD_CONSTANT;``` and the compiler will work out the value of LOOKAHEAD_RADIUS before runtime (Bare in mind you can do this for **entire functions** as well as variables).

### Smart Pointers

A lot of ROS2's messages are generally built upon shared pointers (a type of smart pointer), so understanding pointers in general, and then a bit about smart pointers, is particularly useful.

Check out this Daves garage video for a good rundown on shared pointers:

- [https://www.youtube.com/watch?v=Tp5-f9YAzNk](https://www.youtube.com/watch?v=Tp5-f9YAzNk)

And here is a pretty good video about smart pointers and the differences between them:

- [https://www.youtube.com/watch?v=UOB7-B2MfwA](https://www.youtube.com/watch?v=UOB7-B2MfwA)

## Daves garage: C++ tips and tricks

Generally videos are pretty specific but very understandable and interesting. I found him useful for learning new syntax I didn't know about before, not necessarily for learning the language from scratch.

- [https://youtu.be/p-sprvJX07E?si=VT_AmNzlIyPxxL-O](https://youtu.be/p-sprvJX07E?si=VT_AmNzlIyPxxL-O)

## The Cherno: C++ tutorials

Bite sized videos on different syntaxes and concepts in C++

- [https://youtu.be/18c3MTX0PK0?si=WufdgWOuxfEqoX4p](https://youtu.be/18c3MTX0PK0?si=WufdgWOuxfEqoX4p)

### The Construct

The construct is the ROS2 learning domain, heavily gaurded by a massive corporate paywall. Although, they generally do have some videos on their youtube channel. Be aware though, they're generally livestreams, that are at least an hour long, not anywhere near bitesized to say the least.

- [https://www.youtube.com/@TheConstruct](https://www.youtube.com/@TheConstruct)

You might find some stuff on their site as well, but the free content is pretty limited to my knowledge, and might be somewhat adjacent to what you really need to learn to get the most out of the framework.