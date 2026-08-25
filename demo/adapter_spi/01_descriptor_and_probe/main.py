"""Inspect an honest descriptor and prove that probe has no launch side effect."""

from unirobosim_teaching import DESCRIPTOR, TeachingProvider

from unirobosim import WORLD_SCHEMA_VERSION, Provider


def main() -> None:
    provider = TeachingProvider()
    assert isinstance(provider, Provider)
    assert provider.open_count == 0

    report = provider.probe()
    assert report.available
    assert report.descriptor == DESCRIPTOR
    assert WORLD_SCHEMA_VERSION in DESCRIPTOR.supported_world_schema_versions
    assert provider.open_count == 0

    print(f"provider={DESCRIPTOR.provider_id}")
    print(f"capabilities={len(DESCRIPTOR.capabilities)}")
    print("probe_side_effect_free=true")


if __name__ == "__main__":
    main()
