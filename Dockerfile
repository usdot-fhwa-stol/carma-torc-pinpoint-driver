#  Copyright (C) 2024 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Base image for build
ARG DOCKER_ORG="usdotfhwastoldev"
ARG DOCKER_TAG="develop"
FROM ${DOCKER_ORG}/carma-base:${DOCKER_TAG} as base

# Setup stage
FROM base as setup

# Arguments for customization
ARG GIT_BRANCH="develop"
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

# Create source directory and copy files
RUN mkdir -p ~/src
COPY --chown=carma . /home/carma/src/

# Checkout the specified branch
RUN ~/src/docker/checkout.sh -b ${GIT_BRANCH}

# Install dependencies
RUN ~/src/docker/install.sh

# Final stage
FROM base

# Metadata
ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-torc-pinpoint-driver"
LABEL org.label-schema.description="Torc Pinpoint localization driver for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-torc-pinpoint-driver/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Copy artifacts from setup stage
COPY --from=setup /home/carma/install /opt/carma/install

# Make sure permissions are correct
RUN sudo chmod -R +x /opt/carma/install

# Source the ROS setup file
RUN echo "source /opt/carma/install/setup.bash" >> ~/.bashrc

# Default command to run
CMD ["ros2", "launch", "pinpoint", "pinpoint.launch.py", "remap_ns:=/saxton_cav/drivers"]
