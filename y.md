python -m scripts.run_inference_from_opencv_camera \
    -i datasets/custom/recording.mp4 \
    -p "dataset_name=custom model.descriptor_model.model_or_repo=facebookresearch/dinov2:b48308a394a04ccb9c4dd3a1f0a4daa1ce0579b8" \
    -n child_cube_0 child_cube_2 \
    -b rgb \
    -g