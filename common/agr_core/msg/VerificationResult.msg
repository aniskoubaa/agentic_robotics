# Outcome of pre-execution verification.
bool        valid
Violation[] violations
string      oracle_version      # which ruleset produced this — pin it in results
string[]    rules_checked       # rule_ids that actually ran
string[]    rules_not_available # rule_ids that were requested but not implemented
                                # (explicit, so "no violations" is never confused
                                #  with "nothing was checked")
float32     estimated_duration_s
float32     estimated_battery_pct
