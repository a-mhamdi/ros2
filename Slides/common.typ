#import "@preview/gentle-clues:1.2.0": *
#import "@preview/octique:0.1.0": *
#import emoji: quest

// DEF
#let def(txt) = block(
  fill: luma(230),
  inset: 8pt,
  radius: 4pt,
  txt,
)

// EXO
#let c = counter("exo")
#let exo(tlt, txt) = block[
	#c.step()
	#rect(fill: red, radius: 5pt)[*Task #context c.display(): #tlt *] 
	#rect(fill: luma(221))[#txt]
]

// SOLUTION
#let solution(sol) = block[
	#rect(fill: olive.transparentize(75%), radius: 5pt)[#sol]
]
  
// TEST SCENARIO
#let test(tst) = block[
	#rect(fill: red.transparentize(75%), radius: 5pt)[#tst]
	]

#let hl(txt) = block[
		#rect(fill: luma(240), stroke: blue + 1pt, inset: 8pt, radius: 4pt, width: 100%)[#txt]
]

// CODE
#let code(txt, path_filename) = block[
	#octique("mark-github")
	#rect(fill: blue.transparentize(50%), radius: 5pt)[#txt]
	#link("https://github.com/a-mhamdi/ros2/blob/main/" + path_filename)[#path_filename]
]
