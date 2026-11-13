FROM xtrainer-astra:isaac-2.1.0@sha256:6deeed4a37c67b8a737c8de9e0ecc6c049dbf60ed93b83c24e9dbc2c83a1ebad AS warp-fixed

USER root
COPY repair.py /tmp/isaac-repair/repair.py
ADD https://codeload.github.com/NVIDIA/warp/tar.gz/refs/tags/v1.8.1 /tmp/isaac-repair/warp-source.tar.gz
ADD https://files.pythonhosted.org/packages/fd/68/11e342d24932c187b315fbd1dcf9164c54f936fecacd2d9fa644b4fb19dd/warp_lang-1.8.1-py3-none-manylinux_2_28_x86_64.whl /tmp/isaac-repair/warp_lang-1.8.1-py3-none-manylinux_2_28_x86_64.whl
RUN /isaac-sim/python.sh /tmp/isaac-repair/repair.py warp

FROM warp-fixed AS repaired
ADD https://download.pytorch.org/whl/cu118/torchvision-0.20.1%2Bcu118-cp310-cp310-linux_x86_64.whl /tmp/isaac-repair/torchvision-0.20.1+cu118-cp310-cp310-linux_x86_64.whl
RUN /isaac-sim/python.sh /tmp/isaac-repair/repair.py torchvision && rm -rf /tmp/isaac-repair
LABEL rm27.runtime.warp="1.8.1" rm27.runtime.torchvision="0.20.1+cu118" \
      rm27.runtime.base="sha256:6deeed4a37c67b8a737c8de9e0ecc6c049dbf60ed93b83c24e9dbc2c83a1ebad"
