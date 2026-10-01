# WPILib 2027 alpha-7 for Codespaces and local dev containers.
#
# Everything a student needs to build, test and simulate is baked in here, so a
# fresh Codespace runs ./gradlew build without downloading anything:
#   - the exact JDK the WPILib alpha-7 installer ships (Temurin 25.0.4.1+1),
#     at the path the WPILib VS Code extension looks for it
#   - the WPILib VS Code extension .vsix, which is not on the Marketplace
#   - a warm Gradle cache: the Gradle distribution, GradleRIO, WPILib, Phoenix 6,
#     the Linux x86-64 simulation natives and halsim_gui
#   - the X11 and Mesa libraries halsim_gui needs to draw on the noVNC desktop
#
# The build context is the repository root (see devcontainer.json), so the
# cache is warmed against this branch's own build.gradle and vendordeps.
FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

ARG WPILIB_YEAR=2027_alpha7
ARG JDK_VERSION=25.0.4.1_1
ARG JDK_TAG=jdk-25.0.4.1%2B1
ARG WPILIB_EXT_VERSION=2027.0.0-alpha-7

# halsim_gui is GLFW + OpenGL. On the noVNC desktop there is no GPU, so Mesa's
# llvmpipe software renderer draws it.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libgl1 libglx-mesa0 libgl1-mesa-dri libegl1 \
        libx11-6 libxext6 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

ENV WPILIB_HOME=/home/vscode/.local/share/wpilib/${WPILIB_YEAR}
ENV JAVA_HOME=${WPILIB_HOME}/jdk
ENV PATH=${JAVA_HOME}/bin:${PATH}

# The JDK, checked against Adoptium's published SHA-256.
RUN set -eux; \
    base="https://github.com/adoptium/temurin25-binaries/releases/download/${JDK_TAG}"; \
    file="OpenJDK25U-jdk_x64_linux_hotspot_${JDK_VERSION}.tar.gz"; \
    curl -fsSL -o /tmp/jdk.tgz "${base}/${file}"; \
    echo "$(curl -fsSL "${base}/${file}.sha256.txt" | cut -d' ' -f1)  /tmp/jdk.tgz" | sha256sum -c -; \
    mkdir -p "${JAVA_HOME}"; \
    tar -xzf /tmp/jdk.tgz --strip-components=1 -C "${JAVA_HOME}"; \
    rm /tmp/jdk.tgz; \
    mkdir -p /opt/wpilib; \
    curl -fsSL -o /opt/wpilib/vscode-wpilib.vsix \
        "https://github.com/wpilibsuite/vscode-wpilib/releases/download/v${WPILIB_EXT_VERSION}/vscode-wpilib-${WPILIB_EXT_VERSION}.vsix"; \
    chown -R vscode:vscode /home/vscode/.local

USER vscode

# Warm the Gradle cache with this branch's build. build compiles, and
# simulateExternalJava resolves and extracts every desktop simulation native
# (WPILib, Phoenix 6 swsim, halsim_gui, halsim_ds_socket). A branch with no tests
# reports test as NO-SOURCE and never resolves JUnit, so a one-line test is
# added to the throwaway copy to pull it in. The project copy is deleted; only
# ~/.gradle stays in the image.
COPY --chown=vscode:vscode . /tmp/warm
RUN cd /tmp/warm \
    && sed -i 's/\r$//' gradlew && chmod +x gradlew \
    && mkdir -p src/test/java/warm \
    && printf 'package warm;\n\nclass WarmTest {\n  @org.junit.jupiter.api.Test\n  void warm() {}\n}\n' \
        > src/test/java/warm/WarmTest.java \
    && ./gradlew --no-daemon build test simulateExternalJava \
    && cd / && rm -rf /tmp/warm
