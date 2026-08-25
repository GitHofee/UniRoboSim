"""Show that unsupported capabilities fail explicitly instead of returning fake data."""

from unirobosim_teaching import TeachingProvider

from unirobosim import (
    CapabilityId,
    CapabilityNegotiationError,
    CapabilityRequirement,
    EntityKind,
    EntityPath,
    EntitySpec,
    UnsupportedCapabilityError,
    WorldSpec,
)


def main() -> None:
    entity = EntitySpec(EntityPath("/arm"), EntityKind.ARTICULATION, joint_names=("joint",))
    unsupported = CapabilityRequirement(CapabilityId("sensor.camera.rgb@1"), reason="model observation")

    session = TeachingProvider().open()
    try:
        try:
            session.build(WorldSpec("unsupported-build", (entity,), requirements=(unsupported,)))
        except CapabilityNegotiationError as error:
            assert error.backend_id == "example.teaching"
            print(f"build_rejected={error.code}")
        else:
            raise AssertionError("an unsupported required capability must reject the build")

        world = session.build(WorldSpec("supported-build", (entity,)))
        handle = world.resolve(EntityPath("/arm"))
        try:
            world.read_sensor(handle)
        except UnsupportedCapabilityError as error:
            assert error.details["capability"] == "sensor.camera@1"
            print(f"endpoint_rejected={error.code}")
        else:
            raise AssertionError("an unsupported endpoint must not fabricate a value")
        world.close()
    finally:
        session.close()


if __name__ == "__main__":
    main()
