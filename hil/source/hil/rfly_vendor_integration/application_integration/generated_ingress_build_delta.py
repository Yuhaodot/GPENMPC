"""Narrow HOST-only adjudication of the queue8 -> queue16 generator output."""
import hashlib
from pathlib import Path


def adjudicate(link, build):
    sha = lambda raw: hashlib.sha256(raw).hexdigest().upper()
    build = Path(build)
    generated = build / 'msg/topics_sources/gpenmpc_full_inner_ingress.cpp'
    header = build / 'uORB/topics/gpenmpc_full_inner_ingress.h'
    message = build.parent / 'external/msg/GPENMPCFullInnerIngress.msg'
    assert link['passed'] is False and link['sources_stable'] is False
    assert not link.get('first_exception')
    assert link['original_source_anchors_unchanged'] is True
    assert link['linked_checks'] and all(v is True for v in link['linked_checks'].values())
    assert not link['retired_selected_sources']
    assert {r['label'] for r in link['commands']} == {'APPLICATION', 'SYMBOLS'}
    assert all(r['exit'] == 0 for r in link['commands'])
    actual = {p: sha(Path(p).read_bytes()) for p in link['sources_before']}
    changed = [p for p, h in link['sources_before'].items() if actual[p] != h]
    assert changed == [str(generated)], 'Only the generated ingress metadata may change'
    raw = generated.read_bytes()
    before = b'), 8);'
    after = b'), 16);'
    assert raw.count(after) == 1 and before not in raw
    inverse = raw.replace(after, before)
    assert sha(inverse) == link['sources_before'][str(generated)], 'Any other generated change is forbidden'
    assert sha(message.read_bytes()) == '496BF49D9CF8F1759A93798253D88A6488F5610598244F7C8B9ED98C3D9DF9D1'
    header_bytes = header.read_bytes()
    assert b'#define GPENMPC_FULL_INNER_INGRESS_ORB_QUEUE_LENGTH 16' in header_bytes
    assert b'static constexpr uint8_t ORB_QUEUE_LENGTH = 16;' in header_bytes
    return dict(passed=True,
        classification='BUILD_LINK_PASSED__EXPECTED_GENERATED_QUEUE_DELTA_ADJUDICATED__ORIGINAL_RESULT_PRESERVED',
        actual_link_and_nm_exit_zero=True, all_linked_checks_true=True,
        original_source_anchors_unchanged=True, other_selected_sources_unchanged=True,
        source_count=len(actual), changed_source_count=1,
        changed_source=dict(path=str(generated), before_sha256=link['sources_before'][str(generated)],
                            after_sha256=actual[str(generated)],
                            exact_inverse='Replace the sole metadata suffix ), 16); with ), 8);; SHA must equal original.'),
        message=dict(path=str(message), sha256=sha(message.read_bytes())),
        header=dict(path=str(header), sha256=sha(header_bytes))), actual
