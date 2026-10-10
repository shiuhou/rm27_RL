FROM sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14
ARG HTTP_PROXY
ARG HTTPS_PROXY
RUN DEBIAN_FRONTEND=noninteractive apt-get --no-upgrade -o Acquire::Retries=2 install -y --no-install-recommends gdb && dpkg-query -W > /debug-dependencies.tsv
