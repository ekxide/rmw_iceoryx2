// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/attribute_specifier.hpp"
#include "iox2/backpressure_strategy.hpp"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "rmw_iceoryx2_cxx/impl/qos/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace
{

using ::rmw::iox2::Convert;
using ::rmw::iox2::ProfileKind;
using ::rmw::iox2::Qos;
using ::rmw::iox2::QosError;
using ::rmw::iox2::TryConvert;
namespace attributes = ::rmw::iox2::qos::attributes;

class QosTest : public ::testing::Test
{
};

// ----------------------------------------------------------------------------
// Rejection
// ----------------------------------------------------------------------------

TEST_F(QosTest, resolve_rejects_unknown_history) {
    auto profile = rmw_qos_profile_default;
    profile.history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), QosError::UNKNOWN_POLICY);
}

TEST_F(QosTest, resolve_rejects_unknown_reliability) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), QosError::UNKNOWN_POLICY);
}

TEST_F(QosTest, resolve_rejects_unknown_durability) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), QosError::UNKNOWN_POLICY);
}

TEST_F(QosTest, resolve_rejects_unknown_liveliness) {
    auto profile = rmw_qos_profile_default;
    profile.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), QosError::UNKNOWN_POLICY);
}

TEST_F(QosTest, resolve_rejects_keep_all) {
    auto profile = rmw_qos_profile_default;
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), QosError::UNSUPPORTED_HISTORY_POLICY);
}

// ----------------------------------------------------------------------------
// System defaults
// ----------------------------------------------------------------------------

TEST_F(QosTest, resolve_substitutes_system_default_depth) {
    auto profile = rmw_qos_profile_default;
    profile.depth = RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().depth(), 10U);
}

TEST_F(QosTest, resolve_substitutes_system_default_reliability) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().reliability(), Qos::Reliability::RELIABLE);
}

TEST_F(QosTest, resolve_substitutes_system_default_durability) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().durability(), Qos::Durability::VOLATILE);
}

TEST_F(QosTest, resolve_substitutes_system_default_liveliness) {
    auto profile = rmw_qos_profile_default;
    profile.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().liveliness(), Qos::Liveliness::AUTOMATIC);
}

// ----------------------------------------------------------------------------
// Best available
// ----------------------------------------------------------------------------

TEST_F(QosTest, resolve_substitutes_best_available_reliability) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().reliability(), Qos::Reliability::RELIABLE);
}

TEST_F(QosTest, resolve_substitutes_best_available_durability) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().durability(), Qos::Durability::VOLATILE);
}

TEST_F(QosTest, resolve_substitutes_best_available_deadline) {
    auto profile = rmw_qos_profile_default;
    profile.deadline = {9223372036ULL, 854775806ULL};
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().deadline().sec, 0U);
    EXPECT_EQ(result.value().deadline().nsec, 0U);
}

// ----------------------------------------------------------------------------
// Pass-through
// ----------------------------------------------------------------------------

TEST_F(QosTest, resolve_passes_through_best_effort) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().reliability(), Qos::Reliability::BEST_EFFORT);
}

TEST_F(QosTest, resolve_passes_through_transient_local) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().durability(), Qos::Durability::TRANSIENT_LOCAL);
}

TEST_F(QosTest, resolve_passes_through_manual_by_topic) {
    auto profile = rmw_qos_profile_default;
    profile.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().liveliness(), Qos::Liveliness::MANUAL_BY_TOPIC);
}

TEST_F(QosTest, resolve_passes_through_non_default_depth) {
    auto profile = rmw_qos_profile_default;
    profile.depth = 50;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().depth(), 50U);
}

TEST_F(QosTest, resolve_passes_through_deadline_lifespan) {
    auto profile = rmw_qos_profile_default;
    profile.deadline = {5, 250};
    profile.lifespan = {10, 500};
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().deadline().sec, 5U);
    EXPECT_EQ(result.value().deadline().nsec, 250U);
    EXPECT_EQ(result.value().lifespan().sec, 10U);
    EXPECT_EQ(result.value().lifespan().nsec, 500U);
}

TEST_F(QosTest, resolve_passes_through_liveliness_lease_duration) {
    auto profile = rmw_qos_profile_default;
    profile.liveliness_lease_duration = {3, 100};
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().liveliness_lease_duration().sec, 3U);
    EXPECT_EQ(result.value().liveliness_lease_duration().nsec, 100U);
}

// ----------------------------------------------------------------------------
// Backpressure mapping
// ----------------------------------------------------------------------------

TEST_F(QosTest, enables_safe_overflow_for_reliable) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result.value().enable_safe_overflow());
}

TEST_F(QosTest, enables_safe_overflow_for_best_effort) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result.value().enable_safe_overflow());
}

TEST_F(QosTest, uses_non_blocking_backpressure_strategy_for_reliable) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().backpressure_strategy(), ::iox2::BackpressureStrategy::DiscardData);
}

TEST_F(QosTest, uses_non_blocking_backpressure_strategy_for_best_effort) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().backpressure_strategy(), ::iox2::BackpressureStrategy::DiscardData);
}

TEST_F(QosTest, resolve_passes_through_avoid_ros_namespace_conventions) {
    auto profile = rmw_qos_profile_default;
    profile.avoid_ros_namespace_conventions = true;
    auto result = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result.value().avoid_ros_namespace_conventions());
}

// ----------------------------------------------------------------------------
// To rmw profile
// ----------------------------------------------------------------------------

TEST_F(QosTest, produces_rmw_default_profile_from_default_qos) {
    auto resolved = TryConvert<Qos>::from(rmw_qos_profile_default, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved.has_value());

    auto back = Convert<rmw_qos_profile_t>::from(resolved.value());
    EXPECT_EQ(back.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(back.depth, 10U);
    EXPECT_EQ(back.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(back.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    EXPECT_EQ(back.liveliness, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    EXPECT_FALSE(back.avoid_ros_namespace_conventions);
}

TEST_F(QosTest, convert_to_rmw_best_effort_transient_local) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto resolved = TryConvert<Qos>::from(profile, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved.has_value());

    auto back = Convert<rmw_qos_profile_t>::from(resolved.value());
    EXPECT_EQ(back.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    EXPECT_EQ(back.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

// ----------------------------------------------------------------------------
// Attribute round-trip
// ----------------------------------------------------------------------------

void expect_roundtrip(const rmw_qos_profile_t& profile, ProfileKind kind = ProfileKind::PUBLISH_SUBSCRIBE) {
    auto original = TryConvert<Qos>::from(profile, kind);
    ASSERT_TRUE(original.has_value());

    auto specifier = TryConvert<::iox2::AttributeSpecifier>::from(original.value());
    ASSERT_TRUE(specifier.has_value());

    auto roundtripped = TryConvert<Qos>::from(specifier.value().attributes(), kind);
    ASSERT_TRUE(roundtripped.has_value());

    const auto& expected = original.value();
    const auto& actual = roundtripped.value();
    EXPECT_EQ(expected.history(), actual.history());
    EXPECT_EQ(expected.depth(), actual.depth());
    EXPECT_EQ(expected.reliability(), actual.reliability());
    EXPECT_EQ(expected.durability(), actual.durability());
    EXPECT_EQ(expected.deadline().sec, actual.deadline().sec);
    EXPECT_EQ(expected.deadline().nsec, actual.deadline().nsec);
    EXPECT_EQ(expected.lifespan().sec, actual.lifespan().sec);
    EXPECT_EQ(expected.lifespan().nsec, actual.lifespan().nsec);
    EXPECT_EQ(expected.liveliness(), actual.liveliness());
    EXPECT_EQ(expected.liveliness_lease_duration().sec, actual.liveliness_lease_duration().sec);
    EXPECT_EQ(expected.liveliness_lease_duration().nsec, actual.liveliness_lease_duration().nsec);
    // avoid_ros_namespace_conventions is intentionally NOT round-tripped through attributes
}

TEST_F(QosTest, roundtrip_default_profile) {
    expect_roundtrip(rmw_qos_profile_default);
}

TEST_F(QosTest, roundtrip_sensor_data_profile) {
    expect_roundtrip(rmw_qos_profile_sensor_data);
}

TEST_F(QosTest, roundtrip_services_default) {
    expect_roundtrip(rmw_qos_profile_services_default, ProfileKind::SERVICE);
}

TEST_F(QosTest, roundtrip_large_depth) {
    auto profile = rmw_qos_profile_default;
    profile.depth = 500;
    expect_roundtrip(profile);
}

TEST_F(QosTest, roundtrip_transient_local) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    expect_roundtrip(profile);
}

TEST_F(QosTest, roundtrip_with_deadline_and_lifespan) {
    auto profile = rmw_qos_profile_default;
    profile.deadline = {1, 500'000};
    profile.lifespan = {2, 0};
    expect_roundtrip(profile);
}

TEST_F(QosTest, roundtrip_manual_liveliness_with_lease) {
    auto profile = rmw_qos_profile_default;
    profile.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    profile.liveliness_lease_duration = {3, 100};
    expect_roundtrip(profile);
}

// ----------------------------------------------------------------------------
// Missing attributes
// ----------------------------------------------------------------------------

TEST_F(QosTest, rejects_empty_attribute_set) {
    ::iox2::AttributeSpecifier empty_spec;
    auto result = TryConvert<Qos>::from(empty_spec.attributes(), ProfileKind::PUBLISH_SUBSCRIBE);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), QosError::ATTRIBUTE_DECODING_FAILURE);
}

// ----------------------------------------------------------------------------
// Mismatch detection
// ----------------------------------------------------------------------------

namespace
{
struct CapturedMismatch
{
    std::string key;
    std::string requested;
    std::string existing;
};

template <typename Attribute>
void check(const Qos& qos, ::iox2::AttributeSetView attribute_set, std::vector<CapturedMismatch>& diffs) {
    char requested[256];

    Attribute::encode(qos, requested, sizeof(requested));
    attributes::visit_attribute_value(attribute_set, Attribute::KEY, [&](const char* value) {
        if (std::strcmp(requested, value) != 0) {
            diffs.push_back(CapturedMismatch{Attribute::KEY, requested, value});
        }
    });
}

auto collect_mismatches(const Qos& qos, ::iox2::AttributeSetView attribute_set) -> std::vector<CapturedMismatch> {
    std::vector<CapturedMismatch> mismatches;

    check<attributes::History>(qos, attribute_set, mismatches);
    check<attributes::Reliability>(qos, attribute_set, mismatches);
    check<attributes::Durability>(qos, attribute_set, mismatches);
    check<attributes::Deadline>(qos, attribute_set, mismatches);
    check<attributes::Lifespan>(qos, attribute_set, mismatches);
    check<attributes::Liveliness>(qos, attribute_set, mismatches);

    return mismatches;
}
} // namespace

TEST_F(QosTest, mismatch_reports_none_when_attributes_match) {
    auto resolved = TryConvert<Qos>::from(rmw_qos_profile_default, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved.has_value());

    auto spec = TryConvert<::iox2::AttributeSpecifier>::from(resolved.value());
    ASSERT_TRUE(spec.has_value());

    auto diffs = collect_mismatches(resolved.value(), spec.value().attributes());
    EXPECT_TRUE(diffs.empty());
}

TEST_F(QosTest, mismatch_reports_reliability_difference) {
    auto profile_a = rmw_qos_profile_default;
    profile_a.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    auto resolved_a = TryConvert<Qos>::from(profile_a, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_a.has_value());

    auto profile_b = rmw_qos_profile_default;
    profile_b.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto resolved_b = TryConvert<Qos>::from(profile_b, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_b.has_value());

    auto spec_b = TryConvert<::iox2::AttributeSpecifier>::from(resolved_b.value());
    ASSERT_TRUE(spec_b.has_value());

    auto diffs = collect_mismatches(resolved_a.value(), spec_b.value().attributes());
    ASSERT_EQ(diffs.size(), 1u);
    EXPECT_EQ(diffs[0].key, "rmw.qos.local.reliability");
    EXPECT_EQ(diffs[0].requested, "reliable");
    EXPECT_EQ(diffs[0].existing, "best_effort");
}

TEST_F(QosTest, mismatch_reports_history_depth_difference) {
    auto profile_a = rmw_qos_profile_default;
    profile_a.depth = 5;
    auto resolved_a = TryConvert<Qos>::from(profile_a, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_a.has_value());

    auto profile_b = rmw_qos_profile_default;
    profile_b.depth = 20;
    auto resolved_b = TryConvert<Qos>::from(profile_b, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_b.has_value());

    auto spec_b = TryConvert<::iox2::AttributeSpecifier>::from(resolved_b.value());
    ASSERT_TRUE(spec_b.has_value());

    auto diffs = collect_mismatches(resolved_a.value(), spec_b.value().attributes());
    ASSERT_EQ(diffs.size(), 1u);
    EXPECT_EQ(diffs[0].key, "rmw.qos.local.history");
    EXPECT_EQ(diffs[0].requested, "keep_last:5");
    EXPECT_EQ(diffs[0].existing, "keep_last:20");
}

TEST_F(QosTest, mismatch_reports_durability_difference) {
    auto profile_a = rmw_qos_profile_default;
    profile_a.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    auto resolved_a = TryConvert<Qos>::from(profile_a, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_a.has_value());

    auto profile_b = rmw_qos_profile_default;
    profile_b.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto resolved_b = TryConvert<Qos>::from(profile_b, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_b.has_value());

    auto spec_b = TryConvert<::iox2::AttributeSpecifier>::from(resolved_b.value());
    ASSERT_TRUE(spec_b.has_value());

    auto diffs = collect_mismatches(resolved_a.value(), spec_b.value().attributes());
    ASSERT_EQ(diffs.size(), 1u);
    EXPECT_EQ(diffs[0].key, "rmw.qos.local.durability");
    EXPECT_EQ(diffs[0].requested, "volatile");
    EXPECT_EQ(diffs[0].existing, "transient_local");
}

TEST_F(QosTest, mismatch_reports_multiple_diffs_in_schema_order) {
    // Mismatches on history, reliability, durability — three distinct
    // policies. Verify the callback fires once per mismatch in schema order.
    auto profile_a = rmw_qos_profile_default;
    profile_a.depth = 5;
    profile_a.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    profile_a.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    auto resolved_a = TryConvert<Qos>::from(profile_a, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_a.has_value());

    auto profile_b = rmw_qos_profile_default;
    profile_b.depth = 20;
    profile_b.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    profile_b.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto resolved_b = TryConvert<Qos>::from(profile_b, ProfileKind::PUBLISH_SUBSCRIBE);
    ASSERT_TRUE(resolved_b.has_value());

    auto spec_b = TryConvert<::iox2::AttributeSpecifier>::from(resolved_b.value());
    ASSERT_TRUE(spec_b.has_value());

    auto diffs = collect_mismatches(resolved_a.value(), spec_b.value().attributes());
    ASSERT_EQ(diffs.size(), 3u);
    EXPECT_EQ(diffs[0].key, "rmw.qos.local.history");
    EXPECT_EQ(diffs[1].key, "rmw.qos.local.reliability");
    EXPECT_EQ(diffs[2].key, "rmw.qos.local.durability");
}

} // namespace
