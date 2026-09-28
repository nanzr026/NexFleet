# Factory asset dependency audit

The WSL Webots R2025a installation lacks the requested factory PROTO files. Upstream R2025a definitions were inspected before choosing an implementation:

- [WoodenPallet.proto](https://github.com/cyberbotics/webots/blob/R2025a/projects/objects/factory/pallet/protos/WoodenPallet.proto) depends on the RoughPine appearance PROTO.
- [CardboardBox.proto](https://github.com/cyberbotics/webots/blob/R2025a/projects/objects/factory/containers/protos/CardboardBox.proto) depends on the Cardboard appearance PROTO.

The active world uses original project-local Pallet.proto and WarehouseBox.proto with native geometry and local appearances. This avoids installation-specific appearance/texture downloads. Pallets have individual boards and support blocks; cartons have tape, a seam and shipping-label detail. No upstream asset code or texture is copied here.
