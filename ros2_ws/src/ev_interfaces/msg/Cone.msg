string color     # "YELLOW"(왼쪽 기대) 또는 "BLUE"(오른쪽 기대)
string side      # 화면상 실제 위치 판정: "LEFT" | "RIGHT" | "CENTER"
bool side_ok      # side가 기대한 쪽과 일치하거나 CENTER면 true (반대쪽이면 오검출/역주행 의심)
float32 cx        # 이미지 내 콘 중심 x (pixel)
float32 cy        # 이미지 내 콘 중심 y (pixel)
float32 area      # 컨투어 면적 (pixel^2) - 클수록 카메라에 가까움
