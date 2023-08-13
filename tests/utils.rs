#[cfg(test)]
mod tests {
    use raspberry_pi_localization::utils::Utils;

    #[test]
    fn test_parse_to_rmc() {
        let sentence =
            "$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_some());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_none());

        let sentence = "abcdefg";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_none());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_some());

        let sentence = "$GNRMC,202521.36,V,,,,,,,090823,,,N,V*1A\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_some());
    }
}
