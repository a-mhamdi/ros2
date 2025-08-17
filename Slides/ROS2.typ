#import "@preview/touying:0.6.1": *
#import themes.metropolis: *
#import "@preview/numbly:0.1.0": numbly
#import "@preview/codly:1.3.0": *
#import "@preview/codly-languages:0.1.8": *
#import "@preview/cetz:0.3.4"

#import "@preview/theorion:0.3.2": *
#import cosmos.clouds: *
#show: show-theorion

#show link:underline

#set document(
  title: "ROS",
  author: "A. Mhamdi",
  // keywords: ***,
  date: auto
)

#let code-style = text.with(
  font: "New Computer Modern Mono",
  size: 9pt
)
#set text(font: "Delicious", size: 20pt)
#show math.equation: set text(font: "TeX Gyre Termes Math", size:20pt)

#set par(justify: true)

/*
  #show outline.entry: it => link(
    it.element.location(),
    it.indented(it.prefix(), it.inner()),
  )
*/

#show: codly-init.with()
#codly(languages: codly-languages)

#show: metropolis-theme.with(
  aspect-ratio: "16-9",
  footer: self => "ROS | A. Mhamdi",
  config-info(
    title: [Robot Operating Systems],
    subtitle: [Robot Software Development],
    author: [Abdelbacet Mhamdi],
    date: datetime.today(),
    institution: [MT \@ ISET Bizerte],
  ),
)

#set heading(numbering: numbly("{1}.", default: "1.1"))

#title-slide()

= Outline <touying:hidden>
#outline(title: none, depth: 1)

#include "parts/foundations.typ"            // ROS2 Foundations
#include "parts/nodes-com.typ"              // Nodes and Communication
// #include "parts/interface-dev.typ"          // Custom Interface Development
// #include "parts/nav-perception.typ"         // Navigation and Perception
// #include "parts/adv-ctl.typ"                // Advanced Control Systems
// #include "parts/prod-deployment.typ"        // Production Deployment Strategies

// #include "parts/old/aFile.typ"

#focus-slide[
  Thank you for your attention
]

// #show: appendix

// Bibliography
#set heading(numbering: none, outlined: false) 
= Bibliography

---

#bibliography("bibliography.bib", full: true, style: "apa", title: none)
