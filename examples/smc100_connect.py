import logging
from typing import Optional
from newport_smc100.smc100 import SMC100, ControllerState, HomeSearchType


def main() -> None:
    """

    :return: None
    """
    smc100: Optional[SMC100] = None

    try:
        smc100: SMC100 = SMC100(controller_address=1)
        smc100.connect(port="COM4")
        logging.info(f"Identity : {smc100.get_id()}")

        logging.info(f"Revision information : {smc100.get_revision_information()}")

        logging.info(f"Configuration parameters : {smc100.get_all_configuration_parameters()}")

        # smc100.get_controller_status()
        # logging.info(
        #     "Positioner Error Readable : "
        #     f"{smc100.get_positioner_errors_readable(error_code=int(smc100.positioner_error, 16))}"
        # )
        # logging.info(f"controller_state : {smc100.controller_state}")
        #
        # # smc100.initialize()
        # # smc100.home(search_type=HomeSearchType.MZ_ONLY)
        # # smc100.wait_for_end_of_homing()
        #
        # logging.info(f"position : {smc100.get_position()}")
        # logging.info(f"velocity : {smc100.get_velocity()}")
        # logging.info(f"acceleration : {smc100.get_acceleration()}")
        #
        # smc100.move_absolute(pos=8.0)
        # smc100.wait_for_end_of_motion()
        # logging.info(f"position : {smc100.get_position()}")
        #
        # smc100.move_relative(pos=-4.0)
        # smc100.wait_for_end_of_motion()
        # logging.info(f"position : {smc100.get_position()}")

        # logging.info(f"Encoder Increment : {smc100.get_encoder_increment_value()}")

    except Exception as e:
        raise e

    finally:
        if smc100 is not None:
            smc100.tear()


if __name__ == "__main__":
    # Start logging
    logging.basicConfig(
        # level=logging.DEBUG,
        level=logging.INFO,
        format="%(asctime)s:%(module)s:%(levelname)s - %(message)s"
    )

    main()
