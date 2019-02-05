# literature

## overview visualization methods

- ! [[Hur15] Image-Based Visualization Interactive Multidimensional Data Exploration (2015)](https://ieeexplore.ieee.org/xpl/ebooks/bookPdfWithBanner.jsp?fileName=7374860.pdf&bkn=7374859&pdfType=book)
    - book
    - edge-bundling?
- [[AAB+11] A conceptual framework and taxonomy of techniques for analyzing movement (2011)](https://www.sciencedirect.com/science/article/pii/S1045926X11000139)
    - describe in systematical and comprehensive way the possible types of information that can be extracted from movement data
    - Spatio-temporal data, in particular, movement data, involve geographical space, time, various objects existing and occurring in space, and multidimensional attributes changing over time
- [[SM00] Visualisierung : Grundlagen und allgemeine Methoden (2000)](https://katalogbeta.slub-dresden.de/id/0000139508/#detail)
    - book at slub
 

## Generalized tubes (and cylinder) theory

- !! [[SGS05] Visualization with stylized line primitives (2005)](https://ieeexplore.ieee.org/document/1532859/)
    - 3D vector field vis
    - extended line primitives with additional attributes: color, width, texture, orientation
    - stylized line primitives as generalized cylinders
    - shadow algorithm for depth perception
- ! [[GM03] High Performance generalized cylinders visualization (2003)](https://ieeexplore.ieee.org/abstract/document/1199625/)
    - real-time rendering of highly deformable generalized cylinders
    - efficient schemes for high axis curvature
    - hardware based rendering using skinning
- [[KPL94] Modeling and Animation of Generalized Cylinders with Variable Radius Offset Space Curves (1994)](http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.39.7336)
    - mathematical basics for GC
        + Frenet moving frame
        + Cross-sectional curves & profile curves
        + boundary surface of GC
    - e.g. modeling a tree with variable radius cross-sections (not circle)
- [[NvdP98] Rendering Generalized Cylinders with Paintstrokes (1998)](http://graphicsinterface.org/proceedings/gi1998/gi1998-29/)
    - efficient technique for dynamically tessellating generalized cylinders
    - A-Buffer approach (standard in in transparent line and tube rendering)
- [[AB76] Computer Description of Curved Objects (1976)](https://ieeexplore.ieee.org/document/1674626/)
    - introduction of GC

## Particle (discrete object) trajectory Visualization (3D)

- [[LSK+17] Trajectory Mapper: Interactive Widgets and Artist-Designed Encodings for Visualizing Multivariate Trajectory Data (2017)](http://ivlab.cs.umn.edu/generated/pub-Lange-2017-Trajectory-Mapper.php)
    - interactive mapping of visible attributes (since number of attributes typically larger than number of visible attributes)
- [[SGG17] Temporal Focus+Context for Clusters in Particle Data (2017)](https://diglib.eg.org/handle/10.2312/vmv20171263)
    - combines particle animation and abstraction
    - abstraction: illustrative techniques (ribbons)
    - using screen alligned ribbon as drawing area for line plots
- ! [[KVLR+17] Visual Analysis of Stochastic Trajectory Ensembles in Organic Solar Cell Design (2017)](http://www.mdpi.com/2227-9709/4/3/25)
    - data: stochastic particle trajectory ensembles
    - related work: overview
    - also 3D data
    - encoding information onto trajectory: color, thickness, arrows
- [Visual Analysis of Trajectories in Multi-Dimensional State Spaces (2014)](http://www.joules.de/files/grottel_visual_2014.pdf)
    - visualization of multi-dimensional data combined in linked views
    - trajectories represent different data values of one single entity at different points in time
- [[FW12] Motion Visualization of Large Particle Simulations (2012)](https://wwwcg.in.tum.de/research/research/publications/2012/motion-visualization-of-particle-simulations.html)
    - hierarchical space-time data structure for particle sets
- !! [[WAPW06] Visualizing the underwater behavior of humpback whales (2006)](https://ieeexplore.ieee.org/document/1652919/)
    - ribbon visualization with additional attributes (texture and glyphs)
- !! [[SKH+05] Virtual Tubelets—efficiently visualizing large amounts of particle trajectories(2005)](https://www.sciencedirect.com/science/article/pii/S0097849304001906)
    - tubes


## Trajectory Visualization in context of Flow Visualization

- !! [[GRT13] Opacity Optimization for 3D Line Fields (2013)](http://wwwisg.cs.uni-magdeburg.de/visual/files/publications/2013/Guenther_2013_TOG.pdf)
    - trajectories of surface particle (figure 7: dice example)
- !! [[EHS13] LineAO—Improved Three-Dimensional Line Rendering (2013)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6216373)
    - for flow visualization and tractography
    - rendering large number of dense lines
    - illuminated tubes and lines
    - ambient occlusion and global lighting
- ! [[MLP+10] Over Two Decades of Integration‐Based, Geometric Flow Visualization (2010)](https://onlinelibrary.wiley.com/doi/full/10.1111/j.1467-8659.2010.01650.x)
    - overview geometric flow vis (particle placed in flow)
    - challenges: perception (clutter, occlusion, visual complexity), conveyance of depth and spatial orientation
    - stream lines, stream balls, stream polygons, stream arrows, stream surface
- ! [[EBRI09] Depth-Dependent Halos: Illustrative Rendering of Dense Line Data (2009)](https://ieeexplore.ieee.org/document/5290742/)
    - illustrative rendering for lines
- ! [[Wei07] GPU-Based Interactive Visualization Techniques (2007)](https://katalogbeta.slub-dresden.de/id/0006349644/#detail)
    - [google-books version](https://books.google.de/books?id=Kw4LHkXcKC8C&printsec=frontcover&hl=de&source=gbs_ge_summary_r&cad=0#v=onepage&q&f=false)
    - book at slub, exemplar at CG
    - visualization of 3D scalar fields and vector field visualization
        - stream-tubes
    - pages 87-95 sparse and dense representation for Particle-Tracing techniques
- [[MSE+06] Hybrid Visualization for White Matter Tracts using Triangle Strips and Point Sprites (2006)](https://ieeexplore.ieee.org/document/4015480/)
    - textured triangle strips and point sprites for tube-like appearance
- [[RBE+06] GPU-Based Hyperstreamlines for Diffusion Tensor Imaging (2006)](https://www.researchgate.net/publication/220778370_GPU-Based_Hyperstreamlines_for_Diffusion_Tensor_Imaging)
    - render hyperstreamlines with tublet primitives (supporting local rotations)
- [[KKKW05] A particle system for interactive visualization of 3D flows (2005)](https://ieeexplore.ieee.org/abstract/document/1512024/)
    - data: 3D flow fields
    - GPU-based algorithm for interactive streaming and rendering of millions of particles
    - vis as stream lines and stream bands, stream ribbons to display additional visual clues
- [[MPSS05] Illuminated streamlines revisited (2005)](https://ieeexplore.ieee.org/abstract/document/1532772/)
    - improved spatial perception of lines
    - combination of illuminated streamlines and cylinder averaging
- [[USM96] Efficient streamline, streamribbon, and streamtube constructions on unstructured grids (1996)](https://ieeexplore.ieee.org/document/506222/)
    - definitions of streamribbon and streamtubes + algos
    - streamribbon: streamline + normal vector
    - streamtubes as "iconic streamtubes": streamline + circular crossflow sections along streamline
- [[BHR+94] Streamball techniques for flow visualization (1994)](https://ieeexplore.ieee.org/document/346315/)
    - creating tubes by blending streamballs together
- [[DH93] Visualizing second-order tensor fields with hyperstreamlines (1993)](https://ieeexplore.ieee.org/document/219447/)
    - hyperstreamlines are characterized by the geometry of their cross sections that swepts along the trajectory
- [[SVL91] The stream polygon-a technique for 3D vector field visualization (1991)](https://ieeexplore.ieee.org/document/175789/)
    - stream polygon can be swept along the stream line to form a tube


## cartography based visualizations (2D)

- !! [PWPC18] Assessing the Graphical Perception of Time and Speed on 2D+Time Trajectories (2018)](http://innovis.cpsc.ucalgary.ca/supplemental/2DTimeTrajectories/)
    - analyzes different techniques for encoding of time and speed onto 2D trajectories
- ! [[AA13] Visual analytics of movement: an overview of methods, tools, and procedures (2013)](https://pdfs.semanticscholar.org/cde0/855cb750d15790c7dbd1f657a90fae9d2630.pdf)
    - cited by alot of other publications
    - cartography based
- [[GAM13] Visualization Techniques of Trajectory Data: Challenges and Limitations (2013)](https://www.researchgate.net/publication/289868527_Visualization_techniques_of_trajectory_data_Challenges_and_limitations)
    - trajectories on maps
- [[TSAA12] Stacking-Based Visualization of Trajectory Attribute Data (2012)](https://ieeexplore.ieee.org/document/6327262/)
    - data: 2D discrete object trajectories
    - avoid cluttering by stacking
    - stacked visualization (2D-trajectories over time) with additional information (speed as color)
- [[AMST11] Visualization of time-oriented data (2011)](https://katalogbeta.slub-dresden.de/id/0015205092/#detail)
    - book at slub
    - overview of mutlti-dimensional time-depentend data
    - available chapter: survey of visualization techniques: space-time-cube, spatio-temporal events, space-time paths, geo-time


## Projects and Frameworks

- [glTrajectory (2017)](https://github.com/eozd/glTrajectory)
    - visualize 3D trajectory data using python and OpenGL
- [Superplot3d: an open source GUI tool for 3d trajectory visualisation and elementary processing (2013)](https://scfbm.biomedcentral.com/articles/10.1186/1751-0473-8-19)
    - opensource MATLAB script
    - generates trajectory from 3D-point and time --> line
    - no halos, ambient occlusion or global lighting
- [[WTW+13] FlowVisual: Design and Evaluation of a Visualization Tool for Teaching 2D Flow Field Concepts (2013)](https://www3.nd.edu/~cwang11/2dflowvis.html)
    - overview 2D flow: streamlines, pathlines, streaklines, timelines
- [[SML06] VTK Textbook (2006)](https://www.vtk.org/vtk-textbook/)
    - stream lines, stream ribbons and stream polygons