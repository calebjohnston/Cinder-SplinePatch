Cinder-SplinePatch
==================

This project contains an implementation of a [B-Spline](https://en.wikipedia.org/wiki/B-spline) patch for Cinder v0.9.3. The files are structured as [a CinderBlock](https://libcinder.org/docs/guides/cinder-blocks/index.html) that can be directly incorporated into a new Cinder project. The spline evaluation logic can be found in just two classes, namely [BSplinePatch](src/BSplinePatch.cpp) and [BSplineSurface](src/BSplineSurface.cpp). The BSplinePatch serves as an object initializer for the BSplineSurface. The BSplineSurface inherits from [ci::geom::Source](https://libcinder.org/docs/classcinder_1_1geom_1_1_source.html) so that it can be used to generate OpenGL vertex buffers conveniently. In addition to vertex coordinates, the implementation can optionally generate vertex normals, tangents, bitangents, and texture coordinates.

This project includes two samples that demonstrate use cases and the configuration of a BSpline mesh. The BSplineTest sample shows how the spline patch can be used to smoothly tessellate cylindrical and planar geometry. The Ribbon sample shows how the spline patch can be used creatively to animate visual forms. Both samples rely upon custom shaders for rendering the spline patch geometry.

Currently, all geometry is generated on the CPU and must be uploaded to the GPU each time the patch control points are changed. I hope to support dynamically generated meshes using a tessellation shader in a future version. 
