# Use block outlines for synthetic foreground masks

The synthetic observations will later exercise connector association, which uses
image-region and foreground-mask overlap. Foreground masks must follow the block's
projected CAD outline, preserving background gaps that a filled bounding rectangle
would hide. The CAD visual mesh and enclosing collision box are separate 3D inputs
and cannot be reconstructed from the foreground mask alone.

Inter-object occlusion is outside this change. The supplied scenario keeps blocks
fully in frame and non-overlapping.
