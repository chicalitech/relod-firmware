"""Generate scoped IAM infrastructure for the existing private OTA bucket."""
import json
from pathlib import Path

BUCKET = "arn:aws:s3:::relod-firmware-updates"
ROOT = BUCKET + "/firmware"
PROFILE = "sparkfun_esp32c6_thing_plus/relod-6514-v1"


def statement(actions, resources, condition=None):
    result = {"Effect": "Allow", "Action": actions, "Resource": resources}
    if condition:
        result["Condition"] = condition
    return result


def read_policy():
    # GetObject without ListBucket still fails closed on an unregistered device.
    return [statement(["s3:GetObject"], [ROOT + "/releases/*", ROOT + "/channels/*", ROOT + "/devices/*"]),
            statement(["s3:ListBucket"], [BUCKET], {"StringLike": {"s3:prefix": ["firmware/*"]}})]


def template():
    resources = {
        "GitHubOidc": {"Type": "AWS::IAM::OIDCProvider", "Properties": {
            "Url": "https://token.actions.githubusercontent.com", "ClientIdList": ["sts.amazonaws.com"]}},
        "ApiReadPolicy": {"Type": "AWS::IAM::ManagedPolicy", "Properties": {
            "Description": "Read-only Fly OTA selector; attach to a dedicated runtime identity",
            "PolicyDocument": {"Version": "2012-10-17", "Statement": read_policy()}}},
    }
    outputs = {"ApiReadPolicyArn": {"Value": {"Ref": "ApiReadPolicy"}}}
    for logical, environment, channel in [
        ("CandidateRole", "firmware-candidates", None),
        ("TestRole", "firmware-test", "test"),
        ("ProductionRole", "firmware-production", "stable"),
    ]:
        immutable = [f"{ROOT}/releases/{PROFILE}/*", f"{ROOT}/versions/{PROFILE}/*"] if channel is None else [f"{ROOT}/audits/{PROFILE}/{channel}/*"]
        reads = [f"{ROOT}/releases/{PROFILE}/*", f"{ROOT}/versions/{PROFILE}/*"] if channel is None else [f"{ROOT}/releases/{PROFILE}/*", f"{ROOT}/channels/{PROFILE}/{channel}.json", f"{ROOT}/channels/{PROFILE}/test.json", *immutable]
        statements = [
            statement(["s3:GetObject"], reads),
            # Missing channel GETs need ListBucket to distinguish absent from denied.
            statement(["s3:ListBucket"], [BUCKET]),
            statement(["s3:PutObject"], immutable, {"StringEquals": {"s3:if-none-match": "*", "s3:x-amz-server-side-encryption": "AES256"}}),
        ]
        if channel:
            target = [f"{ROOT}/channels/{PROFILE}/{channel}.json"]
            statements.extend([
                statement(["s3:PutObject"], target, {"StringEquals": {"s3:if-none-match": "*", "s3:x-amz-server-side-encryption": "AES256"}}),
                statement(["s3:PutObject"], target, {"Null": {"s3:if-match": "false"}, "StringEquals": {"s3:x-amz-server-side-encryption": "AES256"}}),
            ])
        resources[logical] = {"Type": "AWS::IAM::Role", "Properties": {
            "MaxSessionDuration": 3600,
            "AssumeRolePolicyDocument": {"Version": "2012-10-17", "Statement": [{
                "Effect": "Allow", "Principal": {"Federated": {"Ref": "GitHubOidc"}},
                "Action": "sts:AssumeRoleWithWebIdentity",
                "Condition": {"StringEquals": {
                    "token.actions.githubusercontent.com:aud": "sts.amazonaws.com",
                    "token.actions.githubusercontent.com:sub": f"repo:chicalitech/relod-firmware:environment:{environment}"}}}]},
            "Policies": [{"PolicyName": "ScopedFirmwareAccess", "PolicyDocument": {"Version": "2012-10-17", "Statement": statements}}]}}
        outputs[logical + "Arn"] = {"Value": {"Fn::GetAtt": [logical, "Arn"]}}
    return {"AWSTemplateFormatVersion": "2010-09-09", "Description": "Relod OTA GitHub OIDC roles; existing bucket and public access settings unchanged", "Resources": resources, "Outputs": outputs}


if __name__ == "__main__":
    Path(__file__).with_name("roles.json").write_text(json.dumps(template(), indent=2) + "\n", encoding="utf-8")
