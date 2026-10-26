# Vitis AI 3.5 (DPUCZDX8G quantizer + compiler) + the ultralytics version used for training.
FROM xilinx/vitis-ai-pytorch-cpu:ubuntu2004-3.5.0.306
RUN bash -lc "source /opt/vitis_ai/conda/etc/profile.d/conda.sh && conda activate vitis-ai-pytorch && \
    pip install --no-cache-dir ultralytics==8.4.185 \"numpy<1.25\" opencv-python-headless==4.10.0.84"
