#[cfg(test)]
mod tests {
    use raspberry_pi_localization::utils::Utils;

    #[test]
    fn test_parse_str_to_float_tuple() {
        let s = "(1.123, 2.799)";
        assert_eq!(Utils::parse_str_to_float_tuple(s), Some((1.123, 2.799)));

        let s = "(999.7223,0.1010)";
        assert_eq!(Utils::parse_str_to_float_tuple(s), Some((999.7223, 0.1010)));

        let s = "(1.2e2, 1e-3)";
        assert_eq!(Utils::parse_str_to_float_tuple(s), Some((120.0, 0.001)));

        let s = "(1.1  100.0)";
        assert_eq!(Utils::parse_str_to_float_tuple(s), None);

        let s = "abcdefg";
        assert_eq!(Utils::parse_str_to_float_tuple(s), None);
    }

    #[test]
    fn test_parse_to_rmc() {
        let sentence = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_some());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_none());

        let sentence = "abcdefg";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_none());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let bytes = sentence.as_bytes().to_vec();
        assert!(Utils::parse_to_rmc(bytes).is_some());
    }
}
