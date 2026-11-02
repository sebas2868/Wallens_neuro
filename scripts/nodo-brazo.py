#!/usr/bin/env python3
"""Lanza el nodo wallens_brazo (necesita ROS cargado y el driver con brazo_habilitado:=true). Lo usa launch/mision.launch.py."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo.nodo import main  # noqa: E402

main()
