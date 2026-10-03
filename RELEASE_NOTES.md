# UniRoboSim 0.10.10

Adds immutable appearance capture/restore contracts for FSR materials, directional lighting, environment, stable visual bindings and portable opaque base-color PNG textures with explicit UV/topology validation. Includes prerequisite deformable render-state/topology and exact float32 per-particle RGBA contracts. Recorded render-state application does not advance physics.

Validation: a Genesis recording was restored into Isaac Lab for all 121 mixed rigid/articulated/deformable/fluid frames; positions/joints/colors matched exactly except root-position float error below 2.4e-8 m. An embedded four-color PNG restored after original texture files were isolated. A real FastSimApplication Replay completed all 76 frames, with 2 materials, 2 bindings and 1 light restored.

Known scope: native output transfer is renderer dependent (Genesis gamma 2.2 versus RTX sRGB), reported through conversion notes. Unsupported material graphs, non-opaque PNG textures, independent emissive materials and unsupported light types are rejected rather than silently approximated. Dynamic appearance mutation is not supported by this release's recording path. The extra FastSimApplication camera subscription currently reports a render-only tick-consistency error; normal Replay completed successfully and SDK RGB output was verified separately. Occasional native Kit startup stalls are bounded by worker startup deadlines.

No Mission, cuRobo, simulator engine or vendor source was changed.

Release checks: 760 CPU tests passed with the new wheel metadata installed in an isolated directory. Wheel and source distribution built successfully.
