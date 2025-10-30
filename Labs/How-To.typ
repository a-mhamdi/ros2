#import "Class.typ": *
#import "Timeline.typ": tml

#set document(keywords: ("ROS2", "Python", "C++", "GNU-Bash", "Typst"), date: auto)

#show: ieee.with(
  title: [#text(smallcaps("ROS2 Lab Report Delivery Tips"))], // ROS2
  abstract: [
		We outline a few rules to adhere to in order to properly prepare your labs. The main programming languages you are going to use to implement some control algorithms in *ROS2* are: Python, C#super[++] and shell scripting using GNU Bash. It is preferable to write your lab reports in Typst, given the provided files.
  ],
  authors:
  (
    (
      name: "Abdelbacet Mhamdi",
      department: [Senior-lecturer, Dept. of EE],
      organization: [Institute of Technological Studies of Bizerte --- Tunisia],
      email: "abdelbacet.mhamdi@bizerte.r-iset.tn",
      profile: "a-mhamdi"
    ),
  ),
  index-terms: ("ROS2", "Python", text[C#super[++]], "GNU-Bash", "Typst"),
  bibliography-file: "Biblio.bib",
)

= Programming Languages

The programming languages we will use in this module are Python (@fig:python), C#super[++] (@fig:cpp), and shell scripting with GNU Bash (@fig:gnu-bash). They are chosen for their expressive syntax, performance where needed, and extensive ecosystems.

/ Python: Rapid development and high-level logic.
/ C#super[++]: Performance and control over system resources.
/ GNU Bash: Scripting and automation of tasks.

#figure(
	image("Images/Python.png", width: 55%),
	caption: [Python logo],
) <fig:python>

#figure(
	image("Images/C++.png", width: 30%),
	caption: [C#super[++] logo],
) <fig:cpp>

#figure(
	image("Images/GNU-Bash.svg", width: 40%),
	caption: [GNU Bash logo],
) <fig:gnu-bash>

Insert your code in each lab report. The detailed explanation of the objects, along the packages you have used is a must. After each code snippet, add a screenshot that showcases your running work when applying the test provided in each exercise.

= Typst
Consider using Typst to write your lab reports. The provided templates allow you to focus on the content and seamlessly create a professional-looking report.

Typst supports Markdown-like syntax, which provides a range of formatting options @Mailund2019. Here are some points to help you write your report:

+	Formatting text:
	-	Bold: surround text with asterisks '(\*)'
	-	Emphasis: surround text with underscores '(\_)'
	-	Headings: start a line with one or more equal signs '(=)'
+	Lists:
	-	Unordered: start with a hyphen '(-)'
	-	Ordered: start with a plus '(+)'
+	Inserting Objects:
  -	Use this syntax if you need to draw a table:
	```typ
	#figure(
		table(
			columns: 4,
			[Row 1], [a], [b], [c],
			[Row 2], [1], [2], [3],
 	),
 	caption: [Results],
	) <tab:LABEL>

	@tab:LABEL displays some results.
	```
	-	Use this syntax if you need to insert an image:
	```typ
	#figure(
		image("IMAGE_NAME.EXT", width: 100%),
		caption: [IMAGE_CAPTION],
	) <fig:LABEL>

	@fig:LABEL shows an image.
	```
+	Code snippets:
	-	Inline: wrap code with backticks (\`)
	-	Block: use triple backticks with a language label (e.g., 'python')
	#raw("```python
import rclpy
from rclpy.node import Node
```")

#reminder

You can leverage those features using the app's intuitive interface and the provided template at the url #highlight[#link("https://typst.app/universe/package/ailab-isetbz")[https://typst.app/universe/package/ailab-isetbz]], as shown in @fig:typstapp. No installation is required; however, you may need to sign up to use the online editor. Keep an eye on your project size. Do not exceed $200$MB. A fully fledged documentation on the usage of Typst is available at #highlight[#link("https://typst.app/docs/")[https://typst.app/docs/]].

#figure(
	image("Images/ailab-isetbz.png", width: 100%),
	caption: [Typst app],
) <fig:typstapp>

= GitHub
Share your code on GitHub. It's a fantastic way to foster a supportive coding community while gaining exposure to different coding styles and techniques @Guthals2023 @Bell2015 @Tsitoara2020.

= Links Bundle
You may find the following links useful:
- GitHub Repository _(@fig:github)_ \ #highlight[#link("https://github.com/a-mhamdi/ros2")[https://github.com/a-mhamdi/ros2]]
#figure(
	image("Images/github-ros2.png", width: 100%),
	caption: [GitHub repository],
) <fig:github>


- Docker Image _(@fig:docker)_ \ #highlight[#link("hub.docker.com/repository/docker/abmhamdi/ros2/general")[hub.docker.com/repository/docker/abmhamdi/ros2]]

#figure(
	image("Images/docker-ros2.png", width: 100%),
	caption: [Docker image],
) <fig:docker>

= Submission Checklist

Before submitting your lab report, verify:

- Code builds and runs (include exact commands)
- Console output and screenshots accompany each task
- Public repository link is provided
- Document uses the Typst template with proper captions for figures/tables

= Timeline

The following timeline is proposed to help you organize your work. It is not mandatory to follow it, but it is highly recommended to do so. The labs are designed to be completed in the order they are presented.

#block(width: 500pt, spacing: 3cm, tml)
// #rotate(-90deg)[#tml]
