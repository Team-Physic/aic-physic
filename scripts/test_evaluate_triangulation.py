# 삼각측량 평가의 실패 집계가 성능을 과장하지 않는지 확인한다.
# 입력은 성공·실패가 섞인 프레임별 mm 오차이며 파일이나 모델을 만들지 않는다.
# 추정이 없을 때 오차를 0으로 보고하거나 실패를 분모에서 빼는 회귀를 막는다.
from evaluate_triangulation import summarize


def test_missing_predictions_are_not_perfect_accuracy():
    """실패 포함 성공률과 유효 추정만의 오차 통계를 구분한다."""
    failed = {'status': 'fewer_than_two_detections', 'error_mm': None}
    result = summarize([{'status': 'ok', 'error_mm': 3.0}, failed])
    assert result['attempts'] == 2 and result['coverage'] == 0.5
    assert result['within_5mm_all_attempts'] == 0.5
    assert result['median_mm'] == 3.0
    assert summarize([failed])['p95_mm'] is None
    assert summarize([])['coverage'] is None
    missing_gt = {'status': 'ground_truth_unavailable', 'error_mm': None}
    result = summarize([failed, missing_gt])
    assert result['input_frames'] == 2 and result['unscorable_frames'] == 1
    assert result['attempts'] == 1 and result['coverage'] == 0.0


if __name__ == '__main__':
    test_missing_predictions_are_not_perfect_accuracy()
    print('Failure denominator and empty-estimate checks passed.')
