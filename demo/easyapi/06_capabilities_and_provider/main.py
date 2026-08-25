"""Declare capabilities and inspect the selected provider."""

from unirobosim import Sim
from unirobosim.testing import FakeProvider


def main() -> None:
    with Sim(provider=FakeProvider(), world_id="capabilities") as sim:
        sim.add_box("box")
        sim.require("state.rigid_body@1", reason="the application reads box state")
        sim.optional("demo.not-supported@1", reason="an optional enhancement")
        sim.start()

        requirements = {item.capability.value: item for item in sim.world_spec.requirements}
        assert sim.provider_descriptor.provider_id == "reference.fake"
        assert requirements["state.rigid_body@1"].required is True
        assert requirements["demo.not-supported@1"].required is False
        assert requirements["demo.not-supported@1"].reason == "an optional enhancement"
        print(f"provider={sim.provider_descriptor.provider_id}")
        for capability_id, requirement in requirements.items():
            print(f"capability={capability_id} required={requirement.required}")


if __name__ == "__main__":
    main()
