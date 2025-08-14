#import "@preview/timeliney:0.3.0"

#import timeliney: *

/* ---------- TIMELINE OF LABS ---------- */
#let tml = (
timeliney.timeline(
  show-grid: true,
  {     
    headerline(group(([*Sept.*], 5)), group(([*Oct.*], 5)), group(([*Nov.*], 5)), group(([*Dec.*], 3)))
    headerline(
      group(..range(5).map(n => strong("W" + str(n + 1)))),
      group(..range(5).map(n => strong("W" + str(n + 1)))),
      group(..range(5).map(n => strong("W" + str(n + 1)))),
      group(..range(3).map(n => strong("W" + str(n + 1)))),

    )
  
    taskgroup(title: [*Lab \#1*], {
      task(text(olive)[_Develop & Code_], (2, 5), style: (stroke: 2pt + olive))
      task(text(maroon)[_Review & Update_], (4, 5), style: (stroke: 2pt + maroon))
      task(text(eastern)[_Finalize & Submit_], (5, 6), style: (stroke: 2pt + eastern))
    })

    taskgroup(title: [*Lab \#2*], {
      task(text(olive)[_Develop & Code_], (4, 7), style: (stroke: 2pt + olive))
      task(text(maroon)[_Review & Update_], (6, 7), style: (stroke: 2pt + maroon))
      task(text(eastern)[_Finalize & Submit_], (7, 8), style: (stroke: 2pt + eastern))
    })

    taskgroup(title: [*Lab \#3*], {
      task(text(olive)[_Develop & Code_], (6, 9), style: (stroke: 2pt + olive))
      task(text(maroon)[_Review & Update_], (8, 9), style: (stroke: 2pt + maroon))
      task(text(eastern)[_Finalize & Submit_], (9, 10), style: (stroke: 2pt + eastern))
    })

    taskgroup(title: [*Lab \#4*], {
      task(text(olive)[_Develop & Code_], (8, 11), style: (stroke: 2pt + olive))
      task(text(maroon)[_Review & Update_], (10, 11), style: (stroke: 2pt + maroon))
      task(text(eastern)[_Finalize & Submit_], (11, 12), style: (stroke: 2pt + eastern))
    })

    taskgroup(title: [*Lab \#5*], {
      task(text(olive)[_Develop & Code_], (10, 13), style: (stroke: 2pt + olive))
      task(text(maroon)[_Review & Update_], (12, 13), style: (stroke: 2pt + maroon))
      task(text(eastern)[_Finalize & Submit_], (13, 14), style: (stroke: 2pt + eastern))
    })

    taskgroup(title: [*Exam*], {
      task(text(lime)[_Review Session_], (14, 16), style: (stroke: 2pt + lime))
      task(text(red)[_Evaluation_], (16, 17), style: (stroke: 2pt + red))
    })

  }
)
)
